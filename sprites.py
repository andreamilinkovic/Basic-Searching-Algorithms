import math
import random
import sys

import pygame
import os
import config

import itertools
import bisect


class BaseSprite(pygame.sprite.Sprite):
    images = dict()

    def __init__(self, x, y, file_name, transparent_color=None, wid=config.SPRITE_SIZE, hei=config.SPRITE_SIZE):
        pygame.sprite.Sprite.__init__(self)
        if file_name in BaseSprite.images:
            self.image = BaseSprite.images[file_name]
        else:
            self.image = pygame.image.load(os.path.join(config.IMG_FOLDER, file_name)).convert()
            self.image = pygame.transform.scale(self.image, (wid, hei))
            BaseSprite.images[file_name] = self.image
        # making the image transparent (if needed)
        if transparent_color:
            self.image.set_colorkey(transparent_color)
        self.rect = self.image.get_rect()
        self.rect.topleft = (x, y)


class Surface(BaseSprite):
    def __init__(self):
        super(Surface, self).__init__(0, 0, 'terrain.png', None, config.WIDTH, config.HEIGHT)


class Coin(BaseSprite):
    def __init__(self, x, y, ident):
        self.ident = ident
        super(Coin, self).__init__(x, y, 'coin.png', config.DARK_GREEN)

    def get_ident(self):
        return self.ident

    def position(self):
        return self.rect.x, self.rect.y

    def draw(self, screen):
        text = config.COIN_FONT.render(f'{self.ident}', True, config.BLACK)
        text_rect = text.get_rect(center=self.rect.center)
        screen.blit(text, text_rect)


class CollectedCoin(BaseSprite):
    def __init__(self, coin):
        self.ident = coin.ident
        super(CollectedCoin, self).__init__(coin.rect.x, coin.rect.y, 'collected_coin.png', config.DARK_GREEN)

    def draw(self, screen):
        text = config.COIN_FONT.render(f'{self.ident}', True, config.RED)
        text_rect = text.get_rect(center=self.rect.center)
        screen.blit(text, text_rect)


class Agent(BaseSprite):
    def __init__(self, x, y, file_name):
        super(Agent, self).__init__(x, y, file_name, config.DARK_GREEN)
        self.x = self.rect.x
        self.y = self.rect.y
        self.step = None
        self.travelling = False
        self.destinationX = 0
        self.destinationY = 0

    def set_destination(self, x, y):
        self.destinationX = x
        self.destinationY = y
        self.step = [self.destinationX - self.x, self.destinationY - self.y]
        magnitude = math.sqrt(self.step[0] ** 2 + self.step[1] ** 2)
        self.step[0] /= magnitude
        self.step[1] /= magnitude
        self.step[0] *= config.TRAVEL_SPEED
        self.step[1] *= config.TRAVEL_SPEED
        self.travelling = True

    def move_one_step(self):
        if not self.travelling:
            return
        self.x += self.step[0]
        self.y += self.step[1]
        self.rect.x = self.x
        self.rect.y = self.y
        if abs(self.x - self.destinationX) < abs(self.step[0]) and abs(self.y - self.destinationY) < abs(self.step[1]):
            self.rect.x = self.destinationX
            self.rect.y = self.destinationY
            self.x = self.destinationX
            self.y = self.destinationY
            self.travelling = False

    def is_travelling(self):
        return self.travelling

    def place_to(self, position):
        self.x = self.destinationX = self.rect.x = position[0]
        self.y = self.destinationX = self.rect.y = position[1]

    # coin_distance - cost matrix
    # return value - list of coin identifiers (containing 0 as first and last element, as well)
    def get_agent_path(self, coin_distance):
        pass


class ExampleAgent(Agent):
    def __init__(self, x, y, file_name):
        super().__init__(x, y, file_name)

    def get_agent_path(self, coin_distance):
        path = [i for i in range(1, len(coin_distance))]
        random.shuffle(path)
        return [0] + path + [0]


class Aki(Agent):
    def __init__(self, x, y, file_name):
        super().__init__(x, y, file_name)

    def get_agent_path(self, coin_distance):
        path = [0]
        next_coin = 0
        for i in range(1, len(coin_distance)):
            tmp = coin_distance[next_coin].copy()
            tmp.sort()
            for j in tmp:
                if (j != 0) and (coin_distance[next_coin].index(j) not in path):
                    next_coin = coin_distance[next_coin].index(j)
                    path.append(next_coin)
                    break

        return path + [0]


class Jocke(Agent):
    def __init__(self, x, y, file_name):
        super().__init__(x, y, file_name)

    def get_agent_path(self, coin_distance):
        paths = list(itertools.permutations(list(range(1, len(coin_distance)))))
        costs = []
        for path in paths:
            cost = 0
            curr_coin = 0
            for next_coin in path:
                cost += coin_distance[curr_coin][next_coin]
                curr_coin = next_coin
            cost += coin_distance[curr_coin][0]
            costs.append(cost)

        path = list(paths[costs.index(min(costs))])
        return [0] + path + [0]


class Uki(Agent):
    def __init__(self, x, y, file_name):
        super().__init__(x, y, file_name)

    def get_agent_path(self, coin_distance):
        paths_list = []
        num_of_coins = len(coin_distance)
        coin_list = list(range(1, num_of_coins))

        for i in range(1, num_of_coins):
            path = list([0, i])
            item = [coin_distance[0][i], len(path), path[len(path) - 1], path]
            index = bisect.bisect_left(paths_list, item)
            paths_list.insert(index, item)

        while 1:
            #print(paths_list)
            #print("\n")

            path, value = (paths_list[0])[3], (paths_list[0])[0]
            paths_list.pop(0)

            if len(path) == num_of_coins + 1:
                break

            curr_coin = path[len(path) - 1]
            if len(path) == num_of_coins:
                path.append(0)
                item = [value + coin_distance[curr_coin][0], len(path), path[len(path) - 1], path]
                index = bisect.bisect_left(paths_list, item)
                paths_list.insert(index, item)
            else:
                coin_remain = list(set(coin_list).difference((set(path))))
                for coin in coin_remain:
                    if coin == 0:
                        continue

                    tmp = path.copy()
                    tmp.append(coin)
                    item = [value + coin_distance[curr_coin][coin], len(tmp), tmp[len(tmp) - 1], tmp]
                    index = bisect.bisect_left(paths_list, item)
                    paths_list.insert(index, item)

        return path


def mst_value(matrix, nodes):
    nodes.append(0)
    num_of_nodes = len(nodes)
    max_nodes = len(matrix)
    num_of_edges = 0
    mst = 0
    visited = [0 for i in range(max_nodes)]
    visited[0] = 1

    while num_of_edges < (num_of_nodes - 1):
        minimum = sys.maxsize
        u = 0
        v = 0
        for m in range(max_nodes):
            if visited[m] and m in nodes:
                for n in range(max_nodes):
                    if not visited[n] and n in nodes:
                        if minimum > matrix[m][n]:
                            minimum = matrix[m][n]
                            u = m
                            v = n
        visited[v] = True
        mst += matrix[u][v]
        num_of_edges += 1

    return mst


class Micko(Agent):
    def __init__(self, x, y, file_name):
        super().__init__(x, y, file_name)

    def get_agent_path(self, coin_distance):
        paths_list = []
        num_of_coins = len(coin_distance)
        coin_list = list(range(1, num_of_coins))

        for i in range(1, num_of_coins):
            path = list([0, i])
            item = [coin_distance[0][i] + mst_value(coin_distance, coin_list), len(path), path[len(path) - 1],
                    coin_distance[0][i], path]
            index = bisect.bisect_left(paths_list, item)
            paths_list.insert(index, item)

        while 1:
            # print(paths_list)
            # print("\n")

            path, h_value, value = (paths_list[0])[4], (paths_list[0])[0], (paths_list[0])[3]
            paths_list.pop(0)

            if len(path) == num_of_coins + 1:
                break

            curr_coin = path[len(path) - 1]
            if len(path) == num_of_coins:
                path.append(0)
                item = [value + coin_distance[curr_coin][0], len(path), path[len(path) - 1],
                        value + coin_distance[curr_coin][0], path]
                index = bisect.bisect_left(paths_list, item)
                paths_list.insert(index, item)
            else:
                coin_remain = list(set(coin_list).difference((set(path))))
                heuristic = mst_value(coin_distance, coin_remain)
                for coin in coin_remain:
                    if coin == 0:
                        continue

                    tmp = path.copy()
                    tmp.append(coin)
                    item = [value + coin_distance[curr_coin][coin] + heuristic, len(tmp), tmp[len(tmp) - 1],
                            value + coin_distance[curr_coin][coin], tmp]
                    index = bisect.bisect_left(paths_list, item)
                    paths_list.insert(index, item)

        return path
