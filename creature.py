import random
import numpy as np
from dm_control import mjcf


class Creature:
    def __init__(self):
        self.model = mjcf.RootElement()
        self.plane = self.model.worldbody.add('geom', type='plane', size=(100, 100, 0.1), pos=(0, 0, -10))
        self.body = self.model.worldbody.add('body', name='body0', pos=(0, 0, 0))
        self.body.add('joint', type='free')
        self.init_body_size = random.uniform(0.1, 1.0)
        self.body.add('geom', type='sphere', size=(self.init_body_size, self.init_body_size, self.init_body_size), pos=(0, 0, self.init_body_size))
        self.body_parts = [self.body]
        self.motors = {}
        self.body_part_dir_used = {}
        self.body_part_dir_used["body0"] = []

    def add_body(self, name, size, pos, axis, body_part, joint_type='hinge'):
        index = self.body_parts.index(body_part)
        body = self.body_parts[index].add('body', name=name, pos=pos)
        body.add('geom', type='sphere', size=(size, size, size))
        body.add('joint', type=joint_type, axis=axis)
        self.body_parts.append(body)
    
    def has_body_in_direction(self, body, direction):
        index = self.body_parts.index(body)
        if direction in self.body_part_dir_used[f"body{index}"]:
            return True
        return False


def find_lowest_body(creature):
    lowest_body = creature.body_parts[0]
    for body in creature.body_parts:
        # print(body.geom[0].pos)
        if body.pos[2] < lowest_body.pos[2]:
            lowest_body = body
    return lowest_body


class CreatureGenerator:
    def __init__(self):
        self.creature = Creature()

    def randomize_creature(self, creature):
        num_bodies = random.randint(2, 15)  # Random limit for number of bodies
        directions = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]  # Possible directions
        for _ in range(num_bodies):
            body_part = random.choice(creature.body_parts)  # Random body part
            direction = random.choice(directions)  # Random direction'
            index = creature.body_parts.index(body_part)
            while creature.has_body_in_direction(body_part, direction):  # Check if direction is occupied
                direction = random.choice(directions)
            size = random.uniform(0.1, 1.0)  # Random size
            parent_body_geom_size = body_part.geom[0].size[0]
            
            creature.body_part_dir_used[f"body{index}"].append(direction)
            name = 'body' + str(len(creature.body_parts))  # Name based on number of body parts
            # add a joint type for a rigid body
            joint_type = random.choice(['hinge', 'ball'])
            # get the geom size of the parent body
            
            creature.add_body(name, size, np.array(direction) * (size + parent_body_geom_size), direction, body_part, joint_type)

            # add used directions for the new body
            creature.body_part_dir_used[name] = []
            creature.body_part_dir_used[name].append(tuple(np.array(direction) * -1))
        
        # for each joint add an actuator
        for joint in creature.model.find_all('joint'):
            motor = creature.model.actuator.add('motor', joint=joint, ctrllimited='true', ctrlrange=(-1, 1))
            # random force between -1 and 1
            random_force = random.uniform(-1, 1)
            creature.motors[motor.name] = random_force
        # update plane position to be 1 unit below the lowest body
        # print(creature.body_parts)
        lowest_body = find_lowest_body(creature)
        creature.plane.pos = (0, 0, lowest_body.pos[2] - lowest_body.geom[0].size[2] - 1)
        return creature

    def generate_creature(self):
        self.creature = self.randomize_creature(self.creature)
        return self.creature