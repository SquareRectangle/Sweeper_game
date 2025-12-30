import pygame
from sys import exit
import math
import numpy

pygame.init()
screen = pygame.display.set_mode((800,400))
pygame.display.set_caption('Runner')
clock = pygame.time.Clock()

class Barrier:
    def __init__(self):
        self.barrier = pygame.Surface((numpy.random.random_integers(10,50), numpy.random.random_integers(10,100)), pygame.SRCALPHA)
        self.barrier.fill('White')
        #self.barrier = pygame.transform.rotate(self.barrier_init, numpy.random.random_integers(0,180))
        self.barrier_rect = self.barrier.get_rect(center = (numpy.random.random_integers(0,800), numpy.random.random_integers(0,400)))

def generate_barriers(spawn_rect):
    barriers = []
    barrier_rects = []
    for i in range(0, numpy.random.random_integers(1,20)):
        i = Barrier()
        if not i.barrier_rect.colliderect(spawn_rect):
            barriers.append(i.barrier)
            barrier_rects.append(i.barrier_rect)
    return barriers, barrier_rects

def barrier_collision(robot_square, rects):
    for rect in rects:
        if robot_square.colliderect(rect) or robot_square.left <= 0 or robot_square.right >=800 or robot_square.top <=0 or robot_square.bottom >=400:
            return True
    return False

def show_barriers(barriers, barrier_rects):
    for i in range(0, len(barriers)):
        screen.blit(barriers[i], barrier_rects[i])

def random_initial_velocity(velocity):
    velocity_x = numpy.random.random() * speed
    velocity_y = numpy.random.choice([1,-1])*math.sqrt(velocity**2 - velocity_x**2)
    return velocity_x, velocity_y

def get_angle_btwn_vectors(v1, v2):
    angle = numpy.degrees(numpy.arccos(numpy.dot(v1, v2)/(numpy.linalg.norm(v2)*numpy.linalg.norm(v1))))
    x_axis_vector = numpy.array([1,0])
    angle_btwn_v_1_and_x_axis = numpy.arccos(numpy.dot(v1, x_axis_vector)/(numpy.linalg.norm(x_axis_vector)*numpy.linalg.norm(v1)))
    angle_btwn_v_2_and_x_axis = numpy.arccos(numpy.dot(v2, x_axis_vector)/(numpy.linalg.norm(x_axis_vector)*numpy.linalg.norm(v2)))
    print(angle_btwn_v_1_and_x_axis, angle_btwn_v_2_and_x_axis)
    if angle_btwn_v_1_and_x_axis < angle_btwn_v_2_and_x_axis:
        return -1*angle
    else:
        return angle

def rotate_robot_to_angle(robot, velocity, initial_direction):
    angle = get_angle_btwn_vectors(initial_direction, velocity)
    print(angle)
    return pygame.transform.rotate(robot, angle)


background = pygame.Surface((800,400))
background.fill('Black')

test_font = pygame.font.Font(None,50)
test_surface = test_font.render("Score:", False, 'Green')

robot_side_length = 20
robot = pygame.Surface((robot_side_length,robot_side_length), pygame.SRCALPHA)
robot.fill('red')

speed = .5
initial_direction = numpy.array([0,1])
velocity_x, velocity_y = random_initial_velocity(speed)
v0 = numpy.array([velocity_x, velocity_y])

#robot transformed is a separate surface that stores all the transformations to the original robot instead of applying the
#transformaitons directly to the robot
robot_transformed = rotate_robot_to_angle(robot, v0, initial_direction)
center = (numpy.random.random_integers(20, 780), numpy.random.random_integers(20,380))
robot_square = robot_transformed.get_rect(center = center)

x_pos = robot_square.centerx
y_pos = robot_square.centery

#sensor beams - initialize the surfaces, fill the color, rotate, and get the rect
center_beam = pygame.Surface((30, 30), pygame.SRCALPHA)
center_beam.fill((255,102,102))
center_beam_transformed = rotate_robot_to_angle(center_beam, v0, initial_direction)
center_beam_rect = center_beam_transformed.get_rect(center = robot_square.center)

beam_width = 25
beam_length = 25

front_beam = pygame.Surface((beam_width, beam_length), pygame.SRCALPHA)
front_beam.fill("Green")
front_beam_transformed = rotate_robot_to_angle(front_beam, v0, initial_direction)
front_beam_rect = front_beam_transformed.get_rect(center = robot_square.center)

#side_beam_width = 15
#side_beam_length = 25

right_beam = pygame.Surface((beam_length,beam_width), pygame.SRCALPHA)
right_beam.fill('Blue')
right_beam_transformed = rotate_robot_to_angle(right_beam, v0, initial_direction)
right_beam_rect = right_beam_transformed.get_rect(center = robot_square.center)

left_beam = pygame.Surface((beam_length,beam_width), pygame.SRCALPHA)
left_beam.fill('Purple')
left_beam_transformed = rotate_robot_to_angle(left_beam, v0, initial_direction)
left_beam_rect = left_beam_transformed.get_rect(center = robot_square.center)



#handle the spawn so that the robot does not spawn in a barrier
spawn = pygame.Surface((30,30), pygame.SRCALPHA)
spawn.fill("Yellow")
spawn = rotate_robot_to_angle(spawn, v0, initial_direction)
spawn_square = spawn.get_rect(center = center)

barriers, barrier_rects = generate_barriers(spawn_square)


while True:

    #print(distance)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()
        if event.type == pygame.MOUSEBUTTONDOWN:
            print("mouse down")
            mouse_x, mouse_y = pygame.mouse.get_pos()

            s_x = mouse_x - x_pos
            s_y = mouse_y - y_pos
            
            #calculate the distance between the origin and the center of the robot
            distance = math.sqrt(s_x**2 + s_y**2)

            #calc velocity components in direction of mouse click
            velocity_x = (speed/distance)*s_x
            velocity_y = (speed/distance)*s_y

            vf = numpy.array([velocity_x, velocity_y])
            #angle  = #numpy.degrees(numpy.arccos(numpy.dot(v0, vf)/(numpy.linalg.norm(vf)*numpy.linalg.norm(v0))))
            #print("angle: ", angle)
            #find the angle of rotation

            robot_transformed = rotate_robot_to_angle(robot, vf, initial_direction)#pygame.transform.rotate(robot, angle)
            robot_square = robot_transformed.get_rect()


            #rotate field to match robot
            front_beam_transformed = rotate_robot_to_angle(front_beam, vf, initial_direction)
            front_beam_rect = front_beam_transformed.get_rect(center = robot_square.center)
            
            right_beam_transformed = rotate_robot_to_angle(right_beam, vf, initial_direction)
            right_beam_rect = right_beam_transformed.get_rect(center = robot_square.center)

            left_beam_transformed = rotate_robot_to_angle(left_beam, vf, initial_direction)
            left_beam_rect = left_beam_transformed.get_rect(center = robot_square.center)

            center_beam_transformed = rotate_robot_to_angle(center_beam, vf, initial_direction)
            center_beam_rect = center_beam_transformed.get_rect(center = robot_square.center)

    x_pos += velocity_x
    y_pos += velocity_y

    #print("vx: ", velocity_x, "vy: ", velocity_y)
    robot_square.centerx = x_pos
    #print("x: ", robot_square.x)
    robot_square.centery = y_pos


    
    #move the fields as well

        #center field
    center_beam_rect.center = robot_square.center

    r = numpy.array(robot_square.center)
    v = robot_side_length/2 + beam_length/2 #17.5               # v defines the length of the vector added to the position vector - the magnitude of the offset
    velocity = numpy.array([velocity_x, velocity_y])
    velocity_hat = velocity/numpy.linalg.norm(velocity)
    
        #front field
    s = r+v*velocity_hat
    field_center = tuple(s)
    front_beam_rect.center = field_center

        #right field
    velocity_3d = numpy.append(velocity, 0)
    z_vector = numpy.array([0,0,1])
    normal_to_velocity = numpy.cross(z_vector, velocity_3d)
    unit_normal = (normal_to_velocity/numpy.linalg.norm(normal_to_velocity))[:2]
    s = r+v*unit_normal
    right_beam_rect.center = tuple(s)

        #left field
    s_left = r-v*unit_normal
    left_beam_rect.center = tuple(s_left)


    

    if barrier_collision(robot_square, barrier_rects):
        print("FAILED")
    if barrier_collision(front_beam_rect, barrier_rects):
        print("FRONT WARNING")
    if barrier_collision(right_beam_rect, barrier_rects):
        print("RIGHT WARNING")
    if barrier_collision(left_beam_rect, barrier_rects):
        print("LEFT WARNING")
    if barrier_collision(center_beam_rect, barrier_rects):
        print("IMPACT IMMENINENT")

    screen.blit(background, (0,0))
    
    show_barriers(barriers, barrier_rects)
    screen.blit(spawn, spawn_square)
    screen.blit(front_beam_transformed, front_beam_rect)
    screen.blit(right_beam_transformed, right_beam_rect)
    screen.blit(left_beam_transformed, left_beam_rect)
    screen.blit(center_beam_transformed, center_beam_rect)
    screen.blit(robot_transformed, robot_square)
    screen.blit(test_surface, (10,10))

    pygame.display.update()
    clock.tick(60) #this while True loop should not run faster than 60 times per second (max frame rate)