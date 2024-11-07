#include <iostream>
using namespace std;

//Base class
class Shape {
public:
    virtual void draw() {
        cout << "Drawing a shape" << endl;
    }
};

// Derived class
class Circle : public Shape {
public:
    void draw() override {
        cout << "Drawing a circle" << endl;
    }
};

// Derived class
class Rectangle : public Shape {
public:
    void draw() override {
        cout << "Drawing a rectangle" << endl;
    }
};

int main() {
    // Creating objects of different classes
    Shape* shape1 = new Shape();
    Circle* circle1 = new Circle();
    Rectangle* rectangle1 = new Rectangle();

    // Polymorphism
    shape1->draw(); // Output: Drawing a shape
    circle1->draw(); // Output: Drawing a circle
    rectangle1->draw(); // Output: Drawing a rectangle

    delete shape1;
    delete circle1;
    delete rectangle1;

    return 0;
}
