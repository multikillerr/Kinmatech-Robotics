"""Standalone FK test utility.
Matplotlib display has been removed; FK results are printed to stdout.
For interactive visualisation use the PyQtGraph visualiser in ui/main.py.
"""
import numpy as np

class RobotVisualizer:
    def __init__(self, dh_params):
        self.dh_params = dh_params
        self.num_axis = dh_params.shape[0]

    def forward_kinematics(self, joint_angles):
        axis_frames = []
        T = np.eye(4)
        for i in range(self.num_axis):
            d, a, alpha, theta = self.dh_params[i]
            theta += joint_angles[i]
            ct, st = np.cos(theta), np.sin(theta)
            ca, sa = np.cos(alpha), np.sin(alpha)
            T_i = np.array([
                [ct, -st * ca,  st * sa, a * ct],
                [st,  ct * ca, -ct * sa, a * st],
                [ 0,       sa,       ca,      d],
                [ 0,        0,        0,      1],
            ])
            T = np.dot(T, T_i)
            axis_frames.append(T.copy())
        return axis_frames

    def update_plot(self, joint_angles):
        """Print FK joint positions (display disabled)."""
        frames = self.forward_kinematics(joint_angles)
        print(f"FK positions for {joint_angles}:")
        print("  base: [0, 0, 0]")
        for i, frame in enumerate(frames):
            pos = frame[:3, 3]
            print(f"  J{i+1}: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")


def load_joint_angles(file_path):
    joint_angles_deg = np.loadtxt(file_path)
    return np.radians(joint_angles_deg)


dh_params = np.array([
    [0.575, 0.175,  0.5 * np.pi, 0.],
    [0.,    0.890,        np.pi, 0.5 * np.pi],
    [0.,    0.050, -0.5 * np.pi, 0.],
    [1.035, 0.,   -0.5 * np.pi, 0.0],
    [0.0,   0.,    0.5 * np.pi, 0.],
    [0.185, 0.,              0., 0.],
])

if __name__ == "__main__":
    robot_visualizer = RobotVisualizer(dh_params)
    try:
        joint_angles_list = load_joint_angles("rotations.txt")
        robot_visualizer.update_plot(joint_angles_list[0])
        print(f"Trajectory has {len(joint_angles_list)} frames.")
    except FileNotFoundError:
        print("rotations.txt not found – printing home position:")
        robot_visualizer.update_plot(np.zeros(6))

    
    def forward_kinematics(self, joint_angles):
        # Compute the forward kinematics using the DH parameters
        axis_frames = []
        T = np.eye(4)  # Initial transformation matrix (identity)
        for i in range(self.num_axis):
            d, a, alpha, theta = self.dh_params[i]
            theta += joint_angles[i]  # Add joint angle to theta
            
            # DH transformation matrix
            T_i = np.array([
                [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                [0, np.sin(alpha), np.cos(alpha), d],
                [0, 0, 0, 1]
            ])
            T = np.dot(T, T_i)
            axis_frames.append(T.copy())
        return axis_frames
    
    def update_plot(self, joint_angles):
        self.ax.clear()
        # Compute the forward kinematics for the given joint angles
        axis_frames = self.forward_kinematics(joint_angles)
        
        # Initialize lists to store x, y, z coordinates of the joints
        x, y, z = [0.], [0.], [0.]
        
        for i in range(self.num_axis):
            frame = axis_frames[i]
            x.append(frame[0, 3])  # x-coordinate of the joint
            y.append(frame[1, 3])  # y-coordinate of the joint
            z.append(frame[2, 3])  # z-coordinate of the joint
        
        # Plot the robot arm
        self.ax.plot(x, y, z, '-o', markersize=8, label='Robot Arm')
        
        # Set plot limits and labels
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([0, 2])
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')
        
        # Draw the plot
        plt.draw()

def load_joint_angles(file_path):
    # Load joint angles from a text file and convert from degrees to radians
    joint_angles_deg = np.loadtxt(file_path)
    joint_angles_rad = np.radians(joint_angles_deg)
    return joint_angles_rad

def update(val):
    # This function will be called whenever the slider is changed
    frame_index = int(slider.val)  # Get the current frame index from the slider
    robot_visualizer.update_plot(joint_angles_list[frame_index])

# Updated DH parameters
dh_params = np.array([[0.575, 0.175, 0.5 * np.pi, 0.],
                      [0., 0.890, np.pi, 0.5 * np.pi],
                      [0., 0.050, -0.5 * np.pi, 0.],
                      [1.035, 0., -0.5 * np.pi, 0.0],
                      [0.0, 0., 0.5 * np.pi, 0.],
                      [0.185, 0., 0., 0.]])

# Create the visualizer with the updated DH parameters
robot_visualizer = RobotVisualizer(dh_params)

# Load the joint angles from the txt file (make sure to replace with the actual path to your file)
joint_angles_list = load_joint_angles("rotations.txt")

# Initialize the robot arm with the first set of joint angles
robot_visualizer.update_plot(joint_angles_list[0])

# Create a slider to control the animation
ax_slider = plt.axes([0.2, 0.02, 0.65, 0.03], facecolor='lightgoldenrodyellow')  # Position of the slider
slider = Slider(ax_slider, 'Frame', 0, len(joint_angles_list) - 1, valinit=0, valstep=1)

# Call the update function when the slider value changes
slider.on_changed(update)

# Display the plot with the slider
plt.show()
