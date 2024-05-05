from vpython import cylinder, vector, color, scene, rate, radians

def create_link(position, length, axis, radius=0.1):
    """Creates a single link as a cylinder."""
    return cylinder(pos=position, axis=axis, radius=radius, color=color.orange)

def rotate_link(link, joint, angle, rotation_axis=vector(0, 0, 1)):
    """Rotates a link around a joint by a specified angle."""
    link.rotate(angle=angle, axis=rotation_axis, origin=joint.pos)

# Set up the initial scene
scene.title = "Simulated Human Finger"
scene.width = 800
scene.height = 600

# Lengths and initial angles of each link
lengths = [2, 1.5, 1]
initial_angles = [0, 0, 0]  # All links initially straight

# Create links
links = []
joints = []
start_position = vector(0, 0, 0)

# Initial setup of links
for i, length in enumerate(lengths):
    if i == 0:
        link_axis = vector(length, 0, 0).rotate(angle=radians(initial_angles[i]), axis=vector(0, 0, 1))
    else:
        link_axis = vector(length, 0, 0).rotate(angle=radians(sum(initial_angles[:i+1])), axis=vector(0, 0, 1))
    link = create_link(start_position, length, link_axis)
    links.append(link)
    # Update the starting position for the next link
    if i < len(lengths) - 1:  # Create joint only if there's another link
        joint = cylinder(pos=link.pos + link.axis, axis=vector(0.2, 0, 0), radius=0.05, color=color.red)
        joints.append(joint)
        start_position = joint.pos

# Function to simulate finger bending
def bend_finger():
    angles = [0, 0, 0]  # Reset angles
    while True:
        rate(30)  # Control speed of animation
        for i in range(1, len(links)):
            if angles[i] < radians(90):  # Increment angle until 90 degrees
                angles[i] += radians(1)
                rotate_link(links[i], joints[i-1], radians(1))

# Start the bending simulation
bend_finger()
