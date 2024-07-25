import cv2


class Button():
    def __init__(self, id_, zone, name, description, text, b_color, t_color, center, radius, hidden):
        self._id = id_
        self._zone = zone
        self._name = name
        self._description = description
        self._text = text
        self._button_color = (b_color.b*255, b_color.g*255, b_color.r*255)
        self._text_color = (t_color.b*255, t_color.g*255, t_color.r*255)
        self._center = (center[0], center[1])
        self._radius = int(radius)
        self._hidden = hidden

    def set_button_color(self,b_color):
        self._button_color = (b_color.b*255, b_color.g*255, b_color.r*255)

    def get_center(self):
        return self._center
    
    def get_zone(self):
        return self._zone
    
    def draw_button(self, img):
        if not self._hidden:
            img = cv2.circle(img, self._center, self._radius, self._button_color, -1)
            TEXT_FACE = cv2.FONT_HERSHEY_DUPLEX
            TEXT_SCALE, text_size, TEXT_THICKNESS = self.get_text_attributes()
            text_origin = (self._center[0] - text_size[0] // 2, self._center[1] + text_size[1] // 2)
            cv2.putText(img, self._text, text_origin, TEXT_FACE, TEXT_SCALE, self._text_color, TEXT_THICKNESS, cv2.LINE_AA)

        return img
    
    def get_text_attributes(self):
        TEXT_FACE = cv2.FONT_HERSHEY_DUPLEX
        TEXT_SCALE = 0.1
        TEXT_THICKNESS = 2
        right_size = False
        text_size, _ = cv2.getTextSize(self._text, TEXT_FACE, TEXT_SCALE, TEXT_THICKNESS)
        if text_size[0] > 60:
            TEXT_THICKNESS = 1
        while not right_size:
            if (text_size[0] < self._radius*2-10 and text_size[0] > self._radius*2-25): 
                right_size = True
            else:
                TEXT_SCALE = TEXT_SCALE + 0.1 
            if TEXT_SCALE > 10:
                right_size = True
            text_size, _ = cv2.getTextSize(self._text, TEXT_FACE, TEXT_SCALE, TEXT_THICKNESS)

        return TEXT_SCALE, text_size, TEXT_THICKNESS
    
    def print_button(self):
        print(self._id)
        print(self._zone)
        print(self._button_color)
