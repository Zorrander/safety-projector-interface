#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

using namespace cv;
using namespace std;

class Instruction {
private:
  int _id;
  int _zone;
  Point _target; // Assuming target is a point; adjust if needed
  string _title;
  Scalar _title_c;
  string _desc;
  Scalar _desc_c;
  int _lifetime;

public:
  Instruction(int id_, int zone, const Point &target, const string &title,
              const Scalar &title_c, const string &description,
              const Scalar &desc_c, int lifetime)
      : _id(id_), _zone(zone), _target(target), _title(title),
        _title_c(title_c * 255), _desc(description), _desc_c(desc_c * 255),
        _lifetime(lifetime) {}

  Mat draw_instruction(Mat &img) const {
    int TEXT_FACE = FONT_HERSHEY_DUPLEX;
    double TEXT_SCALE_TITLE = 2.0;
    double TEXT_SCALE = 1.0;
    int TEXT_THICKNESS = 2;

    int baseline = 0;
    Size text_size_title = getTextSize(_title, TEXT_FACE, TEXT_SCALE_TITLE,
                                       TEXT_THICKNESS, &baseline);
    Size text_size =
        getTextSize(_desc, TEXT_FACE, TEXT_SCALE, TEXT_THICKNESS, &baseline);
    int tot_height = text_size_title.height + text_size.height;

    int start_title_x =
        _target.x + (text_size.width - text_size_title.width) / 2;
    int start_title_y = _target.y - (text_size.height - 10);
    Point pos_desc(_target.x, _target.y + text_size_title.height);

    putText(img, _title, Point(start_title_x, start_title_y), TEXT_FACE,
            TEXT_SCALE_TITLE, _title_c, TEXT_THICKNESS, LINE_AA);
    putText(img, _desc, pos_desc, TEXT_FACE, TEXT_SCALE, _desc_c,
            TEXT_THICKNESS, LINE_AA);

    Point tl_corner(_target.x - 10, _target.y - tot_height);
    Point br_corner(_target.x + text_size.width + 10, _target.y + tot_height);

    // img = rectangle(img, tl_corner, br_corner, _title_c, 2);
    img = draw_border(img, tl_corner, br_corner, _title_c, 2, 10,
                      20); // 80 bracket style

    return img;
  }

private:
  Mat draw_border(Mat &img, const Point &pt1, const Point &pt2,
                  const Scalar &color, int thickness, int r, int d) const {
    int x1 = pt1.x, y1 = pt1.y;
    int x2 = pt2.x, y2 = pt2.y;

    // Top left
    line(img, Point(x1 + r, y1), Point(x1 + r + d, y1), color, thickness);
    line(img, Point(x1, y1 + r), Point(x1, y1 + r + d), color, thickness);
    ellipse(img, Point(x1 + r, y1 + r), Size(r, r), 180, 0, 90, color,
            thickness);

    // Top right
    line(img, Point(x2 - r, y1), Point(x2 - r - d, y1), color, thickness);
    line(img, Point(x2, y1 + r), Point(x2, y1 + r + d), color, thickness);
    ellipse(img, Point(x2 - r, y1 + r), Size(r, r), 270, 0, 90, color,
            thickness);

    // Bottom left
    line(img, Point(x1 + r, y2), Point(x1 + r + d, y2), color, thickness);
    line(img, Point(x1, y2 - r), Point(x1, y2 - r - d), color, thickness);
    ellipse(img, Point(x1 + r, y2 - r), Size(r, r), 90, 0, 90, color,
            thickness);

    // Bottom right
    line(img, Point(x2 - r, y2), Point(x2 - r - d, y2), color, thickness);
    line(img, Point(x2, y2 - r), Point(x2, y2 - r - d), color, thickness);
    ellipse(img, Point(x2 - r, y2 - r), Size(r, r), 0, 0, 90, color, thickness);

    return img;
  }
};
