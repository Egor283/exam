#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class RobotController(Node):
    """
    Узел управления роботом.
    Подписывается на /robot_status и публикует команды скорости в /cmd_vel с частотой 10 Hz.
    Изменяет поведение в зависимости от статуса робота.
    """
    
    # Константы для режимов движения
    MODE_NORMAL = "NORMAL"
    MODE_LOW_BATTERY = "LOW_BATTERY"
    MODE_OBSTACLE = "OBSTACLE_AVOIDANCE"
    MODE_CRITICAL = "CRITICAL_STOP"
    MODE_UNKNOWN = "UNKNOWN"
    
    # Параметры движения для разных режимов
    MOVEMENT_PARAMS = {
        MODE_NORMAL: {
            'linear_x': 0.3,
            'linear_y': 0.0,
            'linear_z': 0.0,
            'angular_x': 0.0,
            'angular_y': 0.0,
            'angular_z': 0.0,
            'description': 'Normal operation'
        },
        MODE_LOW_BATTERY: {
            'linear_x': 0.1,
            'linear_y': 0.0,
            'linear_z': 0.0,
            'angular_x': 0.0,
            'angular_y': 0.0,
            'angular_z': 0.0,
            'description': 'Low battery - reduced speed'
        },
        MODE_OBSTACLE: {
            'linear_x': 0.0,
            'linear_y': 0.0,
            'linear_z': 0.0,
            'angular_x': 0.0,
            'angular_y': 0.0,
            'angular_z': 0.5,
            'description': 'Obstacle detected - turning'
        },
        MODE_CRITICAL: {
            'linear_x': 0.0,
            'linear_y': 0.0,
            'linear_z': 0.0,
            'angular_x': 0.0,
            'angular_y': 0.0,
            'angular_z': 0.0,
            'description': 'Critical - emergency stop'
        },
        MODE_UNKNOWN: {
            'linear_x': 0.0,
            'linear_y': 0.0,
            'linear_z': 0.0,
            'angular_x': 0.0,
            'angular_y': 0.0,
            'angular_z': 0.0,
            'description': 'Unknown status - safe stop'
        }
    }
    
    def __init__(self):
        super().__init__('robot_controller')
        
        # Объявляем пользовательские параметры (НЕ включая use_sim_time)
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('normal_speed', 0.3)
        self.declare_parameter('low_battery_speed', 0.1)
        self.declare_parameter('obstacle_turn_rate', 0.5)
        
        # Получаем пользовательские параметры
        self.update_rate = self.get_parameter('update_rate').value
        self.normal_speed = self.get_parameter('normal_speed').value
        self.low_battery_speed = self.get_parameter('low_battery_speed').value
        self.obstacle_turn_rate = self.get_parameter('obstacle_turn_rate').value
        
        # Получаем системный параметр use_sim_time (не нужно объявлять)
        self.use_sim_time = self.get_parameter_or('use_sim_time', False).value
        
        # Обновляем параметры движения из полученных значений
        self.MOVEMENT_PARAMS[self.MODE_NORMAL]['linear_x'] = self.normal_speed
        self.MOVEMENT_PARAMS[self.MODE_LOW_BATTERY]['linear_x'] = self.low_battery_speed
        self.MOVEMENT_PARAMS[self.MODE_OBSTACLE]['angular_z'] = self.obstacle_turn_rate
        
        # Инициализация состояния
        self.current_status = ""
        self.current_mode = self.MODE_UNKNOWN
        self.last_mode = self.MODE_UNKNOWN
        self.mode_change_count = 0
        self.status_received = False
        self.last_status_time = self.get_clock().now()
        
        # Статистика
        self.commands_published = 0
        self.status_updates = 0
        
        # Настройка QoS для подписки на статус
        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Создаем подписчика на статус робота
        self.status_sub = self.create_subscription(
            String,
            '/robot_status',
            self.status_callback,
            status_qos
        )
        
        # Создаем издателя для команд скорости
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Таймер для периодической публикации команд
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_command
        )
        
        # Таймер для проверки таймаута статуса
        self.timeout_timer = self.create_timer(
            1.0,  # Проверка каждую секунду
            self.check_status_timeout
        )
        
        # Логируем запуск
        self.get_logger().info('Robot Controller initialized')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        self.get_logger().info(f'Normal speed: {self.normal_speed} m/s')
        self.get_logger().info(f'Low battery speed: {self.low_battery_speed} m/s')
        self.get_logger().info(f'Obstacle turn rate: {self.obstacle_turn_rate} rad/s')
        self.get_logger().info(f'Use sim time: {self.use_sim_time}')
        self.get_logger().info(f'Subscribed to /robot_status')
        self.get_logger().info(f'Publishing to /cmd_vel')

    def status_callback(self, msg):
        """
        Обработчик сообщений статуса робота.
        
        Args:
            msg: сообщение со статусом робота
        """
        self.current_status = msg.data
        self.status_received = True
        self.last_status_time = self.get_clock().now()
        self.status_updates += 1
        
        # Определяем режим работы на основе статуса
        new_mode = self.determine_mode(self.current_status)
        
        # Если режим изменился, логируем это
        if new_mode != self.current_mode:
            old_mode = self.current_mode
            self.current_mode = new_mode
            self.mode_change_count += 1
            self.log_mode_change(old_mode, new_mode)
        
        self.get_logger().debug(
            f'Status update #{self.status_updates}: {self.current_status} -> Mode: {self.current_mode}',
            throttle_duration_sec=1.0
        )

    def determine_mode(self, status):
        """
        Определяет режим движения на основе статуса.
        
        Args:
            status: строка со статусом робота
            
        Returns:
            str: режим движения
        """
        if "ALL OK" in status:
            return self.MODE_NORMAL
        elif "WARNING: Low battery" in status:
            return self.MODE_LOW_BATTERY
        elif "WARNING: Obstacle close" in status:
            return self.MODE_OBSTACLE
        elif "CRITICAL" in status:
            return self.MODE_CRITICAL
        else:
            return self.MODE_UNKNOWN

    def check_status_timeout(self):
        """Проверяет, не устарели ли данные статуса."""
        if self.status_received:
            current_time = self.get_clock().now()
            time_since_last = (current_time - self.last_status_time).nanoseconds / 1e9
            
            # Если нет обновлений более 3 секунд, переходим в безопасный режим
            if time_since_last > 3.0 and self.current_mode != self.MODE_UNKNOWN:
                old_mode = self.current_mode
                self.current_mode = self.MODE_UNKNOWN
                self.mode_change_count += 1
                self.get_logger().warn(
                    f'Status timeout! No updates for {time_since_last:.1f}s. '
                    f'Switching from {old_mode} to UNKNOWN mode.'
                )

    def get_current_command(self):
        """
        Возвращает команду движения для текущего режима.
        
        Returns:
            Twist: команда скорости
        """
        cmd = Twist()
        
        # Получаем параметры для текущего режима
        params = self.MOVEMENT_PARAMS.get(
            self.current_mode, 
            self.MOVEMENT_PARAMS[self.MODE_UNKNOWN]
        )
        
        # Заполняем команду
        cmd.linear.x = params['linear_x']
        cmd.linear.y = params['linear_y']
        cmd.linear.z = params['linear_z']
        cmd.angular.x = params['angular_x']
        cmd.angular.y = params['angular_y']
        cmd.angular.z = params['angular_z']
        
        return cmd

    def publish_command(self):
        """Публикует команду скорости."""
        
        # Проверяем, получили ли мы статус
        if not self.status_received:
            self.get_logger().info(
                'Waiting for status... Publishing safe stop commands.',
                throttle_duration_sec=2.0
            )
            # Публикуем нулевую скорость пока нет статуса
            cmd = Twist()
        else:
            # Получаем команду для текущего режима
            cmd = self.get_current_command()
        
        # Публикуем команду
        self.cmd_pub.publish(cmd)
        self.commands_published += 1
        
        # Логирование для отладки
        if self.commands_published % 50 == 0:  # Каждые 5 секунд при 10 Hz
            self.get_logger().debug(
                f'Published cmd #{self.commands_published}: '
                f'linear.x={cmd.linear.x:.2f}, angular.z={cmd.angular.z:.2f}'
            )

    def log_mode_change(self, old_mode, new_mode):
        """Логирует изменение режима работы."""
        
        # Получаем описание нового режима
        description = self.MOVEMENT_PARAMS[new_mode]['description']
        
        # Форматируем сообщение в зависимости от режима
        if new_mode == self.MODE_CRITICAL:
            log_msg = f'🚨 CRITICAL MODE: {description}'
            self.get_logger().error(log_msg)
        elif new_mode in [self.MODE_LOW_BATTERY, self.MODE_OBSTACLE]:
            log_msg = f'⚠️  WARNING MODE: {description}'
            self.get_logger().warn(log_msg)
        elif new_mode == self.MODE_NORMAL:
            log_msg = f'✅ NORMAL MODE: {description}'
            self.get_logger().info(log_msg)
        else:
            log_msg = f'❓ UNKNOWN MODE: {description}'
            self.get_logger().warning(log_msg)
        
        # Дополнительная информация о режиме
        params = self.MOVEMENT_PARAMS[new_mode]
        self.get_logger().info(
            f'  → Command: linear.x={params["linear_x"]:.2f}, '
            f'angular.z={params["angular_z"]:.2f} | '
            f'Status: "{self.current_status}" | '
            f'Mode changes: {self.mode_change_count}'
        )

    def get_mode_statistics(self):
        """
        Возвращает статистику работы контроллера.
        
        Returns:
            dict: словарь со статистикой
        """
        time_since_last = float('inf')
        if self.status_received:
            time_since_last = (self.get_clock().now() - self.last_status_time).nanoseconds / 1e9
            
        return {
            'current_mode': self.current_mode,
            'current_status': self.current_status,
            'mode_changes': self.mode_change_count,
            'commands_published': self.commands_published,
            'status_updates': self.status_updates,
            'status_received': self.status_received,
            'time_since_last_status': time_since_last
        }

    def emergency_stop(self):
        """
        Экстренная остановка робота.
        Может быть вызвана извне.
        """
        old_mode = self.current_mode
        self.get_logger().error('🚨 EMERGENCY STOP ACTIVATED!')
        self.current_mode = self.MODE_CRITICAL
        self.mode_change_count += 1
        self.log_mode_change(old_mode, self.MODE_CRITICAL)
        
        # Немедленно публикуем команду остановки
        cmd = Twist()  # Все нули
        self.cmd_pub.publish(cmd)
        self.commands_published += 1


def main(args=None):
    rclpy.init(args=args)
    
    node = RobotController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # При завершении останавливаем робота
        node.get_logger().info('\n' + '='*50)
        node.get_logger().info('Shutting down Robot Controller...')
        
        # Публикуем команду остановки
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        
        # Выводим статистику
        stats = node.get_mode_statistics()
        node.get_logger().info('Final Statistics:')
        node.get_logger().info(f'  • Final mode: {stats["current_mode"]}')
        node.get_logger().info(f'  • Mode changes: {stats["mode_changes"]}')
        node.get_logger().info(f'  • Commands published: {stats["commands_published"]}')
        node.get_logger().info(f'  • Status updates: {stats["status_updates"]}')
        node.get_logger().info('='*50)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()