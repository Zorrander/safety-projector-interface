#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

class Button {
private:
    int _id;
    int _zone;
    string _name;
    string _description;
    string _text;
    Scalar _button_color;
    Scalar _text_color;
    Point _center;
    int _radius;
    bool _hidden;

public:
    Button(int id_, int zone, const string &name, const string &description, const string &text,
           const Scalar &b_color, const Scalar &t_color, const Point &center, int radius, bool hidden) 
        : _id(id_), _zone(zone), _name(name), _description(description), _text(text), 
          _button_color(b_color * 255), _text_color(t_color * 255), _center(center), _radius(radius), _hidden(hidden) {}

    void set_button_color(const Scalar &b_color) {
        _button_color = b_color * 255;
    }

    Point get_center() const {
        return _center;
    }

    int get_zone() const {
        return _zone;
    }

    Mat draw_button(Mat &img) const {
        if (!_hidden) {
            circle(img, _center, _radius, _button_color, FILLED);

            int TEXT_FACE = FONT_HERSHEY_DUPLEX;
            double TEXT_SCALE;
            Size text_size;
            int TEXT_THICKNESS;

            tie(TEXT_SCALE, text_size, TEXT_THICKNESS) = get_text_attributes();

            Point text_origin(_center.x - text_size.width / 2, _center.y + text_size.height / 2);
            putText(img, _text, text_origin, TEXT_FACE, TEXT_SCALE, _text_color, TEXT_THICKNESS, LINE_AA);
        }
        return img;
    }

    tuple<double, Size, int> get_text_attributes() const {
        int TEXT_FACE = FONT_HERSHEY_DUPLEX;
        double TEXT_SCALE = 0.1;
        int TEXT_THICKNESS = 2;
        bool right_size = false;

        Size text_size;
        int baseline = 0;
        text_size = getTextSize(_text, TEXT_FACE, TEXT_SCALE, TEXT_THICKNESS, &baseline);

        if (text_size.width > 60) {
            TEXT_THICKNESS = 1;
        }

        while (!right_size) {
            if ((text_size.width < _radius * 2 - 10) && (text_size.width > _radius * 2 - 25)) {
                right_size = true;
            } else {
                TEXT_SCALE += 0.1;
            }

            if (TEXT_SCALE > 10) {
                right_size = true;
            }

            text_size = getTextSize(_text, TEXT_FACE, TEXT_SCALE, TEXT_THICKNESS, &baseline);
        }

        return make_tuple(TEXT_SCALE, text_size, TEXT_THICKNESS);
    }

    void print_button() const {
        cout << _id << endl;
        cout << _zone << endl;
        cout << _button_color << endl;
    }
};