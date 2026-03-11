#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class BatteryNode(Node):
    """
    Узел симуляции батареи.
    Публикует уровень заряда в топик /battery_level с частотой 1 Hz.
    Начальный заряд: 100%, разряд: 1% в секунду.
    """
    
    # Константы для уровней логирования
    LOG_LEVELS = [100, 90, 80, 70, 60, 50, 40, 30, 20, 10]
    
    def __init__(self):
        super().__init__('battery_node')
        
        # Объявляем пользовательские параметры
        self.declare_parameter('initial_charge', 100.0)
        self.declare_parameter('discharge_rate', 1.0)  # % в секунду
        self.declare_parameter('update_rate', 1.0)     # Hz
        
        # Получаем пользовательские параметры
        self.initial_charge = self.get_parameter('initial_charge').value
        self.discharge_rate = self.get_parameter('discharge_rate').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Получаем системный параметр use_sim_time (не нужно объявлять)
        self.use_sim_time = self.get_parameter_or('use_sim_time', False).value
        
        # Инициализация состояния батареи
        self.current_charge = self.initial_charge
        self.last_logged_level = int(self.current_charge)
        
        # Настройка QoS для топика батареи
        # Используем transient local durability чтобы последнее значение было доступно новым подписчикам
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Создаем издателя
        self.publisher = self.create_publisher(
            Float32, 
            '/battery_level', 
            qos
        )
        
        # Таймер для периодической публикации
        self.timer = self.create_timer(
            1.0 / self.update_rate, 
            self.publish_battery_level
        )
        
        # Счетчик для дополнительной статистики
        self.publish_count = 0
        
        # Логируем запуск
        self.get_logger().info('Battery node initialized')
        self.get_logger().info(f'Initial charge: {self.initial_charge}%')
        self.get_logger().info(f'Discharge rate: {self.discharge_rate}%/sec')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        self.get_logger().info(f'Use sim time: {self.use_sim_time}')
        self.get_logger().info(f'Publishing to /battery_level')

    def publish_battery_level(self):
        """Публикует текущий уровень заряда батареи."""
        
        # Создаем сообщение
        msg = Float32()
        
        # Проверяем, не разрядилась ли батарея полностью
        if self.current_charge <= 0.0:
            self.current_charge = 0.0
        else:
            # Уменьшаем заряд
            self.current_charge -= self.discharge_rate / self.update_rate
            # Защита от отрицательных значений
            if self.current_charge < 0.0:
                self.current_charge = 0.0
        
        # Устанавливаем значение в сообщение
        msg.data = float(self.current_charge)
        
        # Публикуем сообщение
        self.publisher.publish(msg)
        self.publish_count += 1
        
        # Проверяем, нужно ли логировать текущий уровень
        current_level = int(self.current_charge)
        
        # Логируем при достижении каждого порога в 10%
        # Используем пороги из LOG_LEVELS
        for level in self.LOG_LEVELS:
            if (current_level <= level and 
                self.last_logged_level > level and 
                current_level > 0):
                self.get_logger().info(f'Battery: {level}%')
                self.last_logged_level = level
                break
        
        # Отдельный случай для 0%
        if current_level == 0 and self.last_logged_level > 0:
            self.get_logger().warn('Battery: 0% - Battery depleted!')
            self.last_logged_level = 0
        
        # Логирование для отладки (каждую 10-ю публикацию или при изменении)
        if self.publish_count % 10 == 0 or current_level != int(msg.data):
            self.get_logger().debug(
                f'Battery level: {self.current_charge:.1f}% | '
                f'Published: {self.publish_count} times',
                throttle_duration_sec=5.0
            )

    def get_battery_status(self):
        """
        Возвращает статус батареи.
        Полезно для интеграции с другими узлами.
        """
        if self.current_charge > 20.0:
            return "OK"
        elif self.current_charge > 5.0:
            return "LOW"
        else:
            return "CRITICAL"

    def reset_battery(self):
        """Сбрасывает заряд батареи к начальному значению."""
        self.current_charge = self.initial_charge
        self.last_logged_level = int(self.current_charge)
        self.get_logger().info(f'Battery reset to {self.initial_charge}%')
        
        # Немедленно публикуем новое значение
        self.publish_battery_level()


def main(args=None):
    rclpy.init(args=args)
    
    node = BatteryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Выводим финальный статус при завершении
        node.get_logger().info(
            f'Battery node shutting down. '
            f'Final charge: {node.current_charge:.1f}% | '
            f'Status: {node.get_battery_status()}'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()