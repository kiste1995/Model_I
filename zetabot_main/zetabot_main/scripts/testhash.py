# !/usr/bin/env python

import string
import random

def codecreat():
    _LENGTH = 12
    string_pool = string.ascii_letters + string.digits
      
    result = ""
    for i in range(_LENGTH) :
        result += random.choice(string_pool)
    return result

class hashmap:
    def __init__(self):
        self.size = 15
        self.map = [None] * self.size

    def get_hash(self, key):
        hash_value = 0
        for char in str(key):
            hash_value += ord(char)
        return hash_value % self.size

    def add(self, key, value):
        hash_key = self.get_hash(key)
        hash_value = [key, value]

        if self.map[hash_key] is None:
            self.map[hash_key] = list([hash_value])
            return True
        else:
            for pair in self.map[hash_key]:
                if pair[0] == key:
                    pair[1] = value
                    return True
            self.map[hash_key].append(hash_value)
            return True

    def get(self, key):
        hash_key = self.get_hash(key)

        if self.map[hash_key] is not None:
            for pair in self.map[hash_key]:
                if pair[0] == key:
                    return pair[1]
        return None

    def delete(self, key):
        hash_key = self.get_hash(key)

        if self.map[hash_key] is None:
            return False
        for i in range(0, len(self.map[hash_key])):
            if self.map[hash_key][i][0] == key:
                self.map[hash_key].pop(i)
                return True

    def prints(self):
        for i in self.map:
            print '-------------------------'
            if i is not None:
                for j in i:
                    print j[0], j[1]


