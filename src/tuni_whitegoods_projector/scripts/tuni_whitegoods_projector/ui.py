import numpy as np


#class for interface image
class InterfaceUI():
    def __init__(self, ressource_id, zone, name, description, buttons): #add hidden
        self._ressource_id = ressource_id
        self._zone = zone
        self._name = name
        self._description = description
        self._list_buttons = []
        self._instructions = []
        self._hidden = False
        if len(buttons) > 0:
            self._list_buttons = buttons
        self._screen_size = (1080,1920) 
        self._interface_img = np.zeros((self._screen_size[0], self._screen_size[1], 3), np.uint8)
        self.modified_interface = True

    def add_button(self,button):
        self._list_buttons.append(button)
        self.modified_interface = True

    def add_instruction(self,inst):
        self._instructions.append(inst)
        self.modified_interface = True

    def get_zone(self):
        return self._zone

    def get_list_buttons(self):
        return self._list_buttons
    
    def get_hidden(self):
        return self._hidden

    def draw(self):
        if self.modified_interface and not self._hidden:
            for button in self._list_buttons:
                self._interface_img = button.draw_button(self._interface_img)
            for instruction in self._instructions:
                self._interface_img = instruction.draw_instruction(self._interface_img)
            self.modified_interface = False

        return self._interface_img
    
    def modify_button_color(self,button):
        for i in self._list_buttons:
            if i._id == button.id:
                i.set_button_color(button.button_color)
        self.modified_interface = True

    def get_image_interface(self):
        return self._interface_img
    
    def print_buttons(self):
        for i in self._list_buttons:
            i.print_button()
        