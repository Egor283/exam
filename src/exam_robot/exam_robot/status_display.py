#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class StatusDisplay(Node):
    """
    Узел отображения статуса робота.
    Подписывается на /battery_level и /distance, публикует /robot_status с частотой 2 Hz.
    Определяет статус на основе комбинации показаний датчиков.
    """
    
    # Константы для статусов
    STATUS_ALL_OK = "ALL OK"
    STATUS_LOW_BATTERY = "WARNING: Low battery"
    STATUS_OBSTACLE_CLOSE = "WARNING: Obstacle close"
    STATUS_CRITICAL = "CRITICAL"
    
    # Пороговые значения
    BATTERY_WARNING_THRESHOLD = 20.0
    BATTERY_CRITICAL_THRESHOLD = 10.0
    DISTANCE_WARNING_THRESHOLD = 1.0
    DISTANCE_CRITICAL_THRESHOLD = 0.7
    
    def __init__(self):
        super().__init__('status_display')
        
        # Объявляем параметры
        self.declare_parameter('update_rate', 2.0)  # Hz
        self.declare_parameter('battery_warning', 20.0)
        self.declare_parameter('battery_critical', 10.0)
        self.declare_parameter('distance_warning', 1.0)
        self.declare_parameter('distance_critical', 0.7)
        
        # Получаем параметры
        self.update_rate = self.get_parameter('update_rate').value
        self.battery_warning = self.get_parameter('battery_warning').value
        self.battery_critical = self.get_parameter('battery_critical').value
        self.distance_warning = self.get_parameter('distance_warning').value
        self.distance_critical = self.get_parameter('distance_critical').value
        
        # Инициализация данных датчиков
        self.battery_level = 100.0  # начальное значение
        self.distance = 3.0          # начальное значение
        self.last_status = ""
        self.status_change_count = 0
        
        # Флаги получения данных
        self.received_battery = False
        self.received_distance = False
        self.first_status_published = False
        
        # Настройка QoS для подписок (reliable)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        # Создаем подписчиков
        self.battery_sub = self.create_subscription(
            Float32,
            '/battery_level',
            self.battery_callback,
            qos
        )
        
        self.distance_sub = self.create_subscription(
            Float32,
            '/distance',
            self.distance_callback,
            qos
        )
        
        # Настройка QoS для публикации статуса (transient local для последнего значения)
        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Создаем издателя для статуса
        self.status_pub = self.create_publisher(
            String,
            '/robot_status',
            status_qos
        )
        
        # Таймер для периодической публикации
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_status
        )
        
        # Таймер для проверки отсутствия данных
        self.data_timeout_timer = self.create_timer(
            5.0,  # 5 секунд таймаут
            self.check_data_timeout
        )
        
        # Логируем запуск
        self.get_logger().info('Status Display node initialized')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        self.get_logger().info(f'Battery thresholds: warning={self.battery_warning}%, critical={self.battery_critical}%')
        self.get_logger().info(f'Distance thresholds: warning={self.distance_warning}m, critical={self.distance_critical}m')
        self.get_logger().info(f'Subscribed to /battery_level and /distance')
        self.get_logger().info(f'Publishing to /robot_status')

    def battery_callback(self, msg):
        """Обработчик сообщений батареи."""
        old_level = self.battery_level
        self.battery_level = msg.data
        self.received_battery = True
        
        # Логируем значительные изменения батареи
        if abs(old_level - self.battery_level) >= 5.0:
            self.get_logger().debug(
                f'Battery updated: {old_level:.1f}% -> {self.battery_level:.1f}%',
                throttle_duration_sec=2.0
            )

    def distance_callback(self, msg):
        """Обработчик сообщений дистанции."""
        old_distance = self.distance
        self.distance = msg.data
        self.received_distance = True
        
        # Логируем значительные изменения дистанции
        if abs(old_distance - self.distance) >= 0.3:
            self.get_logger().debug(
                f'Distance updated: {old_distance:.2f}m -> {self.distance:.2f}m',
                throttle_duration_sec=2.0
            )

    def check_data_timeout(self):
        """Проверяет, получаем ли мы данные от датчиков."""
        if not self.received_battery:
            self.get_logger().warn('No battery data received yet', throttle_duration_sec=5.0)
        if not self.received_distance:
            self.get_logger().warn('No distance data received yet', throttle_duration_sec=5.0)

    def determine_status(self):
        """
        Определяет текущий статус робота на основе показаний датчиков.
        
        Returns:
            str: Статус робота
        """
        # Проверяем критические условия (наивысший приоритет)
        if self.battery_level < self.battery_critical or self.distance < self.distance_critical:
            return self.STATUS_CRITICAL
        
        # Проверяем предупреждения
        if self.battery_level < self.battery_warning:
            return self.STATUS_LOW_BATTERY
        
        if self.distance < self.distance_warning:
            return self.STATUS_OBSTACLE_CLOSE
        
        # Если все хорошо
        return self.STATUS_ALL_OK

    def get_status_color(self, status):
        """
        Возвращает цвет для статуса (для красивого вывода в консоль).
        
        Args:
            status: Статус робота
            
        Returns:
            tuple: (color_code, status)
        """
        colors = {
            self.STATUS_ALL_OK: '\033[92m',           # Зеленый
            self.STATUS_LOW_BATTERY: '\033[93m',      # Желтый
            self.STATUS_OBSTACLE_CLOSE: '\033[93m',   # Желтый
            self.STATUS_CRITICAL: '\033[91m'          # Красный
        }
        reset = '\033[0m'
        
        return colors.get(status, '\033[0m'), reset

    def format_status_message(self, status):
        """
        Форматирует сообщение статуса для логирования.
        
        Args:
            status: Статус робота
            
        Returns:
            str: Отформатированное сообщение
        """
        color, reset = self.get_status_color(status)
        
        return (f'{color}Status: {status}{reset} | '
                f'Battery: {self.battery_level:.1f}% | '
                f'Distance: {self.distance:.2f}m')

    def publish_status(self):
        """Публикует текущий статус робота."""
        
        # Проверяем, получили ли мы хотя бы раз данные
        if not self.received_battery or not self.received_distance:
            if not self.first_status_published:
                self.get_logger().info('Waiting for sensor data...')
                return
        
        # Определяем текущий статус
        current_status = self.determine_status()
        
        # Создаем и публикуем сообщение
        msg = String()
        msg.data = current_status
        self.status_pub.publish(msg)
        
        # Проверяем, изменился ли статус
        if current_status != self.last_status:
            self.status_change_count += 1
            
            # Форматируем и логируем изменение статуса
            formatted_msg = self.format_status_message(current_status)
            
            # Разные уровни логирования для разных статусов
            if current_status == self.STATUS_CRITICAL:
                self.get_logger().error(formatted_msg)
            elif current_status in [self.STATUS_LOW_BATTERY, self.STATUS_OBSTACLE_CLOSE]:
                self.get_logger().warn(formatted_msg)
            else:
                self.get_logger().info(formatted_msg)
            
            # Обновляем последний статус
            self.last_status = current_status
            self.first_status_published = True
        
        # Периодическое логирование (каждые 30 секунд) даже если статус не меняется
        elif hasattr(self, 'status_log_counter'):
            self.status_log_counter += 1
            if self.status_log_counter >= 60:  # 60 * 0.5 сек = 30 сек при 2 Hz
                self.get_logger().info(
                    f'Current status: {current_status} | '
                    f'Battery: {self.battery_level:.1f}% | '
                    f'Distance: {self.distance:.2f}m | '
                    f'Changes: {self.status_change_count}',
                    throttle_duration_sec=30.0
                )
                self.status_log_counter = 0
        else:
            self.status_log_counter = 0

    def get_detailed_status(self):
        """
        Возвращает детальную информацию о статусе.
        Полезно для интеграции с другими узлами.
        
        Returns:
            dict: Словарь с детальной информацией
        """
        return {
            'status': self.determine_status(),
            'battery_level': self.battery_level,
            'distance': self.distance,
            'battery_warning': self.battery_level < self.battery_warning,
            'battery_critical': self.battery_level < self.battery_critical,
            'distance_warning': self.distance < self.distance_warning,
            'distance_critical': self.distance < self.distance_critical,
            'changes_count': self.status_change_count
        }


def main(args=None):
    rclpy.init(args=args)
    
    node = StatusDisplay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Выводим финальную статистику при завершении
        node.get_logger().info(
            f'\nStatus Display shutting down.\n'
            f'Final status: {node.last_status}\n'
            f'Status changes: {node.status_change_count}\n'
            f'Battery: {node.battery_level:.1f}%\n'
            f'Distance: {node.distance:.2f}m'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()