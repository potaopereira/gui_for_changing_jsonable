"""Example of Jsonable classes."""

import jsonable

class Pillow(jsonable.Jsonable):
   
    def __init__(self, is_soft=True):
        self.is_soft = is_soft
       

pillow_database = {'Default': Pillow, 'Pillow1': Pillow, 'Pillow2': Pillow}
cushion_database = {'Default': Pillow, 'Cushion1': Pillow, 'Cushion2': Pillow}

class Bed(jsonable.Jsonable):

    inner = {
    'pillow': pillow_database,
    'cushion': cushion_database
    }

    def __init__(self, length=2.0):
        self.length=length
        self.add_inner_defaults()

    @jsonable.add_to_methods_list
    def change_sheets(self,length = 1.0):
        self.length = length

bed_database = {'Default': Bed,'Bed1': Bed, 'Bed2': Bed}

class Room(jsonable.Jsonable):

    inner = {'bed': bed_database}

    def __init__(self, area=18.0):
        self.area=area
        self.add_inner_defaults()

room_database = {'Room1':Room,'Room2':Room}