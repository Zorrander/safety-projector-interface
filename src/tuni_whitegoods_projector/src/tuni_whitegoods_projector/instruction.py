import cv2


#class to draw instructions
class Instruction():
    def __init__(self, id_, zone, target, title, title_c, description, desc_c, lifetime):
        self._id = id_
        self._zone = zone
        self._target = target
        self._title = title
        self._title_c = (title_c.b*255, title_c.g*255, title_c.r*255)
        self._desc = description
        self._desc_c = (desc_c.b*255, desc_c.g*255, desc_c.r*255)
        self._lifetime = lifetime

    def draw_instruction(self,img):
        TEXT_FACE = cv2.FONT_HERSHEY_DUPLEX
        TEXT_SCALE_TITLE = 2.0
        TEXT_SCALE = 1.0
        TEXT_THICKNESS = 2
        text_size_title, _ = cv2.getTextSize(self._title, TEXT_FACE, TEXT_SCALE_TITLE, TEXT_THICKNESS)
        text_size, _ = cv2.getTextSize(self._desc, TEXT_FACE, TEXT_SCALE, TEXT_THICKNESS)
        tot_height = text_size_title[1] + text_size[1]
        start_title_x = int(self._target.position.x) + int((text_size[0] - text_size_title[0]) / 2)
        start_title_y = int(self._target.position.y) - (text_size[1] - 10)
        pos_desc = (int(self._target.position.x),int(self._target.position.y + text_size_title[1]))
        cv2.putText(img, self._title, (start_title_x,start_title_y), TEXT_FACE, TEXT_SCALE_TITLE, self._title_c, TEXT_THICKNESS, cv2.LINE_AA)
        cv2.putText(img, self._desc, pos_desc, TEXT_FACE, TEXT_SCALE, self._desc_c, TEXT_THICKNESS, cv2.LINE_AA)
        tl_corner = (int(self._target.position.x - 10),int(self._target.position.y - tot_height))
        br_corner = (int(self._target.position.x + text_size[0] + 10),int(self._target.position.y + tot_height))
        #img = cv2.rectangle(img, tl_corner, br_corner, self._title_c, 2)
        img = self.draw_border(img,tl_corner,br_corner,self._title_c, 2, 10,20) #80 bracket style

        return img

    def draw_border(self,img, pt1, pt2, color, thickness, r, d):
        x1,y1 = pt1
        x2,y2 = pt2
        # Top left
        cv2.line(img, (x1 + r, y1), (x1 + r + d, y1), color, thickness)
        cv2.line(img, (x1, y1 + r), (x1, y1 + r + d), color, thickness)
        cv2.ellipse(img, (x1 + r, y1 + r), (r, r), 180, 0, 90, color, thickness)
        # Top right
        cv2.line(img, (x2 - r, y1), (x2 - r - d, y1), color, thickness)
        cv2.line(img, (x2, y1 + r), (x2, y1 + r + d), color, thickness)
        cv2.ellipse(img, (x2 - r, y1 + r), (r, r), 270, 0, 90, color, thickness)
        # Bottom left
        cv2.line(img, (x1 + r, y2), (x1 + r + d, y2), color, thickness)
        cv2.line(img, (x1, y2 - r), (x1, y2 - r - d), color, thickness)
        cv2.ellipse(img, (x1 + r, y2 - r), (r, r), 90, 0, 90, color, thickness)
        # Bottom right
        cv2.line(img, (x2 - r, y2), (x2 - r - d, y2), color, thickness)
        cv2.line(img, (x2, y2 - r), (x2, y2 - r - d), color, thickness)
        cv2.ellipse(img, (x2 - r, y2 - r), (r, r), 0, 0, 90, color, thickness)

        return img