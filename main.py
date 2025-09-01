import math
import matplotlib.pyplot as plt


# Lyapunov 
def generate_circle_waypoints(cx, cy, r, n_points):
    waypoints = []
    for i in range(n_points):
        angle = (2*math.pi / n_points) * i
        x = cx + r * math.cos(angle)
        y = cy + r * math.sin(angle)
        theta = angle + math.pi/2  # tangente
        waypoints.append((x, y, theta))
    return waypoints

def generate_tressage_waypoints(x_start, x_end, y_center, amplitude, wavelength, n_points):
    waypoints = []
    for i in range(n_points):
        x = x_start + (x_end - x_start) * (i / (n_points - 1))
        y = y_center + amplitude * math.sin(2 * math.pi * x / wavelength)
        theta = math.atan2(
            2 * math.pi * amplitude / wavelength * math.cos(2 * math.pi * x / wavelength),
            1
        )  # orientation tangentielle
        waypoints.append((x, y, theta))
    return waypoints

waypoints = generate_circle_waypoints(500, 500, 300, 10)

# waypoints = generate_tressage_waypoints(
#     x_start=200, x_end=800, y_center=500, amplitude=100, wavelength=200, n_points=200
# )

# waypoints = [
#     (100, 500, 0),   # Point A, orientation vers la droite
#     (900, 500, math.pi)  # Point B, orientation vers la gauche (retour)
# ]

class Robot:
    def __init__(self, wheel_diameter=8.6, wheel_base=8):
        self.wheel_radius = wheel_diameter / 2
        self.wheel_base = wheel_base
        self.x = 800
        self.y = 400
        self.theta = math.pi/2
        self.left_rpm = 0
        self.right_rpm = 0
        self.traj_x, self.traj_y = [], []
        # Plot
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10,10))
        self.ax.set_xlim(0,1000); self.ax.set_ylim(1000,0)
        self.ax.set_aspect('equal'); self.ax.grid(True, alpha=0.3)
        self.line, = self.ax.plot([], [], 'b-', linewidth=2)
        self.point, = self.ax.plot([], [], 'ro', markersize=8)
        plt.show(block=False)

    def rpm_to_speed(self, rpm): return rpm
    def move(self, left_rpm, right_rpm, dt):
        v = (self.rpm_to_speed(left_rpm)+self.rpm_to_speed(right_rpm))/2
        omega = (self.rpm_to_speed(right_rpm)-self.rpm_to_speed(left_rpm))/self.wheel_base
        self.theta += omega*dt
        self.x += v*math.cos(self.theta)*dt
        self.y += v*math.sin(self.theta)*dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.left_rpm = left_rpm
        self.right_rpm = right_rpm

    def update_plot(self):
        self.traj_x.append(self.x); self.traj_y.append(self.y)
        self.line.set_data(self.traj_x, self.traj_y)
        self.point.set_data([self.x],[self.y])
        self.fig.canvas.draw(); self.fig.canvas.flush_events()

    def limit_accel(current, target, max_delta):
        if target > current + max_delta:
            return current + max_delta
        elif target < current - max_delta:
            return current - max_delta
        return target

def main():
    robot = Robot()
    dt = 0.1
    v_const = 30  # vitesse constante
    k_alpha, k_beta = 2.5, -1.2
    index = 0
    try:
        while True:
            x_target, y_target, theta_target = waypoints[index]
            dx, dy = x_target - robot.x, y_target - robot.y
            rho = math.sqrt(dx**2 + dy**2)
            gamma = math.atan2(dy, dx)
            alpha = math.atan2(math.sin(gamma - robot.theta), math.cos(gamma - robot.theta))
            beta = math.atan2(math.sin(theta_target - robot.theta), math.cos(theta_target - robot.theta))

            v = v_const

            deadband_angle = 0.04  # env. 2°

            if abs(alpha) < deadband_angle:
                omega = 0
            else:
                omega = k_alpha * alpha + k_beta * beta
                max_omega = 1
                omega = max(-max_omega, min(max_omega, omega))

            # omega = k_alpha * alpha + k_beta * beta

            left_rpm = (2*v - omega*robot.wheel_base)/2
            right_rpm = (2*v + omega*robot.wheel_base)/2
            max_rpm = 70
            left_rpm = max(-max_rpm, min(max_rpm, left_rpm))
            right_rpm = max(-max_rpm, min(max_rpm, right_rpm))
            def limit_accel(current, target, max_delta):
                if target > current + max_delta:
                    return current + max_delta
                elif target < current - max_delta:
                    return current - max_delta
                return target
            
            max_delta_rpm = 8  # variation max de RPM par cycle (ajuste selon ton dt)
            left_rpm = limit_accel(robot.left_rpm, left_rpm, max_delta_rpm)
            right_rpm = limit_accel(robot.right_rpm, right_rpm, max_delta_rpm)

            robot.move(left_rpm, right_rpm, dt)

            if rho < 40:  # passage fluide
                index = (index + 1) % len(waypoints)

            robot.update_plot()
            plt.pause(0.001)
    except KeyboardInterrupt:
        plt.ioff(); plt.show()

if __name__ == "__main__":
    main()
