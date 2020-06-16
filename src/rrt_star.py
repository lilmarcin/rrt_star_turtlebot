#!/usr/bin/env python
import rospy as rp
from grid_map import GridMap
import numpy as np


np.random.seed(333)

rodzice = []
distance = []
dlugosc = []
lista = []
widzi =[]
rozmiar_x = 6;
rozmiar_y = 6;

class RRT_STAR(GridMap):
    def __init__(self):
        super(RRT_STAR, self).__init__()
        self.step = 0.2

    def check_if_valid(self, a, b):
        """
        Checks if the segment connecting a and b lies in the free space.
        :param a: point in 2D
        :param b: point in 2D
        :return: boolean
        """
        """
        in_free_space = True
        A = b[1] - a[1]
        B = a[0] - b[0]
        C = A * (a[0]) + B * (a[1])
        if B < 0:
            #print("The line passing through points P and Q is:",A, "x ", B, "y = ", C, "\n")
        else:
            #print("The line passing through points P and Q is: ",A, "x + ", B, "y = ", C, "\n")
        x_min = int(a[0] * 10)
        x_max = int(b[0] * 10)
        y_min = int(a[1] * 10)
        y_max = int(b[1] * 10)
        if x_min > x_max:
            range_x_min = x_max
            range_x_max = x_min
        else:
            range_x_min = x_min
            range_x_max = x_max

        if y_min > y_max:
            range_y_min = y_max
            range_y_max = y_min
        else:
            range_y_min = y_min
            range_y_max = y_max
        print(range_x_min, range_x_max, 'y', range_y_min, range_y_max)
        lista_punktow = []
        for i in range(range_x_min, range_x_max + 1):
            for j in range(range_y_min, range_y_max + 1):
                if C + 0.1 > A * i * 0.1 + B * j * 0.1 > C - 0.1:
                    lista_punktow.append(np.array([i, j]))
        print(lista_punktow)
        for i in lista_punktow:
            if self.map[i[1]][i[0]] == 0:
                in_free_space = True
                print('TRUE')
            else:
                in_free_space = False
                print('FALSE')
                break
        """
        in_free_space = True
        x_range = np.linspace(a[0]+10, b[0]+10, num=1000)
        y_range = np.linspace(a[1]+10, b[1]+10, num=1000)
        for i in range(len(x_range)):
            x = int(x_range[i]*20)
            y = int(y_range[i]*20)
            #print(self.map[383, 383])
            for i in range(-rozmiar_x/2, rozmiar_x/2):
                for j in range(-rozmiar_y / 2, rozmiar_y / 2):
                    obstacle = self.map[y+j, x+i]
                    if obstacle == 100:
                        #print('sciana')
                        in_free_space = False
                    else:
                        pass

        return in_free_space

    def random_point(self):
        """
        Draws random point in 2D
        :return: point in 2D
        """
        #x = np.random.uniform(0., self.width)
        #y = np.random.uniform(0., self.height)
        #x = np.random.uniform(-1., 13.5)	#map_house
        #y = np.random.uniform(-5., 6.)		#map_house
        x = np.random.uniform(-3., 3.)		#map colloseum
        y = np.random.uniform(-3., 3.)		#map colloseum
        return tuple([x, y])

    def find_closest(self, pos):
        """
        Finds the closest vertex in the graph to the pos argument

        :param pos: point id 2D
        :return: vertex from graph in 2D closest to the pos
        """

        for rodzic in rodzice:
            distance.append(np.sqrt((pos[0] - rodzic[0]) ** 2 + (pos[1] - rodzic[1]) ** 2))
        index_min = np.argmin(distance)
        closest = rodzice[index_min]

        return closest

    def find_closest2(self, pos):

        for rodzic in rodzice:
            if self.check_if_valid(pos, rodzic) is True:
                widzi.append(rodzic)
        #print('widzi ',len(widzi))
        if len(widzi) != 0:
            for rodzic in widzi:
                distance1 = rodzic[2]
                distance2 = np.sqrt((pos[0] - rodzic[0]) ** 2 + (pos[1] - rodzic[1]) ** 2)
                distance3 = distance1+distance2
                #print("odleglosc 3 ", distance3)
                distance.append(distance3)
            index_min = np.argmin(distance)
            #print('dist ', len(distance))
            #print('dist ', distance)
            #print('indesx ',index_min)
            closest = widzi[index_min]
            return closest


    def odleglosc(self, pos, stary):

        dlugosc = np.sqrt((pos[0] - stary[0]) ** 2 + (pos[1] - stary[1]) ** 2)
        new = pos + (dlugosc,)

        return new

    def new_pt(self, pt, closest):
        """
        Finds the point on the segment connecting closest with pt, which lies self.step from the closest (vertex in graph)

        :param pt: point in 2D
        :param closest: vertex in the tree (point in 2D)
        :return: point in 2D
        """
        a = abs(closest[0] - pt[0])
        b = abs(closest[1] - pt[1])
        c = np.sqrt((pt[0] - closest[0]) ** 2 + (pt[1] - closest[1]) ** 2)

        sin = float(a / c)
        cos = float(b / c)

        a_new = sin * self.step
        b_new = cos * self.step

        if closest[0] < pt[0] and closest[1] < pt[1]:
            pt = tuple([closest[0] + a_new, closest[1] + b_new])

        elif closest[0] < pt[0] and closest[1] > pt[1]:
            pt = tuple([closest[0] + a_new, closest[1] - b_new])

        elif closest[0] >= pt[0] and closest[1] >= pt[1]:
            pt = tuple([closest[0] - a_new, closest[1] - b_new])

        elif closest[0] > pt[0] and closest[1] < pt[1]:
            pt = tuple([closest[0] - a_new, closest[1] + b_new])

        else:
            pt = tuple([closest[0] + a_new, closest[1] + b_new])

        return pt

    def search(self):
        """
        RRT search algorithm for start point self.start and desired state self.end.
        Saves the search tree in the self.parent dictionary, with key value pairs representing segments
        (key is the child vertex, and value is its parent vertex).
        Uses self.publish_search() and self.publish_path(path) to publish the search tree and the final path respectively.
        """
        self.parent[self.start] = None
        begin = self.start
        #target = self.end
        start_rdy = np.array([begin[1], begin[0]])
        path = []
        child = []
        nowy_begin = begin +(0,)
        rodzice.append(nowy_begin)

        while not rp.is_shutdown():
            del distance[:]
            del widzi[:]
            point = self.random_point()
            #closest = self.find_closest(point)
            closest2 = self.find_closest2(point)

            if self.check_if_valid(point, self.start) is True:
                dlugosc = np.sqrt((point[0] - begin[0]) ** 2 + (point[1] - begin[1]) ** 2)
                nowy_punkt = point + (dlugosc,)
                #nowy_punkt = self.odleglosc(point, stary_punkt)
                print('start :',nowy_punkt)
                rodzice.append(nowy_punkt)
                self.parent[point] = self.start

                if self.check_if_valid(point, self.end) is True:
                    rodzice.append(self.end)
                    self.parent[self.end] = point
                    print("finished")
                    break

            elif closest2 is not None:
                dlugosc = closest2[2] + np.sqrt((point[0] - closest2[0]) ** 2 + (point[1] - closest2[1]) ** 2)
                nowy_punkt = point + (dlugosc,)
                print('child :', nowy_punkt)
                rodzice.append(nowy_punkt)
                kloze = closest2[0:2]
                self.parent[point] = kloze

                finish = self.check_if_valid(point, self.end)
                if finish == True:
                    nowy_punkt = point + (0,)
                    rodzice.append(nowy_punkt)
                    self.parent[self.end] = point
                    print("finished")
                    target = self.end
                    while target is not None:
                        path.append(target)
                        target = self.parent[target]
                    with open("/home/marcinxd/catkin_ws/src/rrt_miapr/src/path.txt",
                              'w') as f:
                        for item in path[::-1]:
                            f.write("{} {}\n".format(item[0], item[1]))
                    break
            rp.sleep(0.01)
            self.publish_search()
        self.publish_search()
        self.publish_path(path)

if __name__ == '__main__':
    rrt = RRT_STAR()
    rrt.search()
