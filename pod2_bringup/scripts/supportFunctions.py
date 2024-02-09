#!/usr/bin/env python3

class myStack:
    def __init__(self):
        self.stack = []

    def add(self, data):
        self.stack.append(data)

    def getTop(self):
        return self.stack[-1]
    
    def pop(self):
        if len(self.stack) == 0: return None
        value = self.stack[-1]
        del self.stack[-1]
        return value

    def __len__(self):
        return len(self.stack) 
    
    def __getitem__(self, index):
        return self.stack[index]