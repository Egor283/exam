#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class DistanceSensor(Node):
    """
    Узел симуляции датчика расстояния.
    Публикует дистанцию до препятствия в топик /distance с частотой 5 Hz.
    Изменяется в зависимости от скорости робота.
    """
    
    def __init__(self):
        super().__init__('distance_sensor')
        
        # Объявляем параметры
        self.declare_parameter('update_rate', 5.0)           # Hz
        self.declare_parameter('max_distance', 3.0)          # метры
        self.declare_parameter('min_distance', 0.5)          # метры
        self.declare_parameter('distance_step', 0.2)         # изменение за шаг
        self.declare_parameter('step_time', 0.2)             # время шага в секундах
        
        # Получаем параметры
        self.update_rate = self.get_parameter('update_rate').value
        self.max_distance = self.get_parameter('max_distance').value
        self.min_distance = self.get_parameter('min_distance').value
        self.distance_step = self.get_parameter('distance_step').value
        self.step_time = self.get_parameter('step_time').value
        
        # Инициализация состояния
        self.current_distance = self.max_distance
        self.current_linear_x = 0.0
        self.last_update_time = self.get_clock().now()
        
        # Настройка QoS для подписки на cmd_vel (лучшая надежность)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        # Создаем подписчика на топик cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos
        )
        self.subscription  # предотвращаем удаление подписчика
        
        # Создаем издателя для дистанции
        self.publisher = self.create_publisher(
            Float32,
            '/distance',
            10
        )
        
        # Таймер для периодической публикации
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_distance
        )
        
        # Счетчик для статистики
        self.publish_count = 0
        self.update_count = 0
        
        # Логируем запуск
        self.get_logger().info('Distance sensor initialized')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        self.get_logger().info(f'Distance range: {self.min_distance}-{self.max_distance}m')
        self.get_logger().info(f'Step: {self.distance_step}m per {self.step_time}s')
        self.get_logger().info(f'Subscribed to /cmd_vel')
        self.get_logger().info(f'Publishing to /distance')

    def cmd_vel_callback(self, msg):
        """
        Обработчик сообщений из топика /cmd_vel.
        Обновляет текущую скорость робота.
        """
        old_speed = self.current_linear_x
        self.current_linear_x = msg.linear.x
        
        # Логируем изменение скорости
        if abs(old_speed - self.current_linear_x) > 0.01:
            direction = "вперед" if self.current_linear_x > 0 else "назад" if self.current_linear_x < 0 else "стоит"
            self.get_logger().info(
                f'Скорость изменена: {old_speed:.2f} -> {self.current_linear_x:.2f} м/с ({direction})',
                throttle_duration_sec=1.0
            )
        
        # Сбрасываем время последнего обновления при смене направления
        if (old_speed > 0 and self.current_linear_x <= 0) or \
           (old_speed < 0 and self.current_linear_x >= 0):
            self.last_update_time = self.get_clock().now()
            self.get_logger().debug('Direction changed, resetting update timer')

    def update_distance(self):
        """
        Обновляет значение дистанции на основе скорости робота.
        Вызывается перед каждой публикацией.
        """
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_update_time).nanoseconds / 1e9  # в секундах
        
        # Сколько шагов прошло с последнего обновления
        steps = int(time_diff / self.step_time)
        
        if steps > 0:
            self.update_count += steps
            
            if abs(self.current_linear_x) < 0.001:  # Робот стоит
                # Плавно возвращаемся к максимальной дистанции
                if self.current_distance < self.max_distance:
                    self.current_distance = min(
                        self.max_distance,
                        self.current_distance + self.distance_step * steps
                    )
                    self.get_logger().debug(f'Returning to max distance: {self.current_distance:.2f}m')
                    
            elif self.current_linear_x > 0:  # Движение вперед
                # Уменьшаем дистанцию
                new_distance = self.current_distance - self.distance_step * steps
                if new_distance < self.min_distance:
                    self.current_distance = self.min_distance
                    self.get_logger().info(f'Достигнут минимум: {self.min_distance}m', throttle_duration_sec=1.0)
                else:
                    self.current_distance = new_distance
                    self.get_logger().debug(f'Moving forward, distance decreased to: {self.current_distance:.2f}m')
                    
            else:  # Движение назад (linear.x < 0)
                # Увеличиваем дистанцию
                new_distance = self.current_distance + self.distance_step * steps
                if new_distance > self.max_distance:
                    self.current_distance = self.max_distance
                    self.get_logger().info(f'Достигнут максимум: {self.max_distance}m', throttle_duration_sec=1.0)
                else:
                    self.current_distance = new_distance
                    self.get_logger().debug(f'Moving backward, distance increased to: {self.current_distance:.2f}m')
            
            # Обновляем время последнего изменения
            self.last_update_time = current_time

    def publish_distance(self):
        """Публикует текущее значение дистанции."""
        
        # Обновляем дистанцию перед публикацией
        self.update_distance()
        
        # Создаем и публикуем сообщение
        msg = Float32()
        msg.data = float(self.current_distance)
        
        self.publisher.publish(msg)
        self.publish_count += 1
        
        # Логирование для отладки (каждую 10-ю публикацию)
        if self.publish_count % 10 == 0:
            direction = "вперед" if self.current_linear_x > 0 else "назад" if self.current_linear_x < 0 else "стоит"
            self.get_logger().info(
                f'Distance: {self.current_distance:.2f}m | '
                f'Speed: {self.current_linear_x:.2f} м/с ({direction}) | '
                f'Updates: {self.update_count}',
                throttle_duration_sec=2.0
            )

    def get_distance_status(self):
        """
        Возвращает статус дистанции.
        Полезно для интеграции с другими узлами.
        """
        if self.current_distance > 2.0:
            return "SAFE"
        elif self.current_distance > 1.0:
            return "CAUTION"
        else:
            return "DANGER"


def main(args=None):
    rclpy.init(args=args)
    
    node = DistanceSensor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Выводим финальный статус при завершении
        node.get_logger().info(
            f'Distance sensor shutting down. '
            f'Final distance: {node.current_distance:.2f}m | '
            f'Status: {node.get_distance_status()} | '
            f'Published: {node.publish_count} messages'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()