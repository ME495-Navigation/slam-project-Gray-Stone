# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- svg - Generate svg file for displaying Point2D, Vector2D and Transform2D
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality

    1. Finds the manitude of Vector2D `mag = (x+y)/2` Then device each element with it `x/mag ; y/mag` 

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

   - Which of the methods would you implement and why?

2. What is the difference between a class and a struct in C++?

class and struct are basically the same in c++, except when member's Access specifiers are not defined, class default to private, struct default to public.

Practically, struct are usually used for simple grouping of related data, while class are for interface, invariant data/concepts, etc.

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?

The purpose of Vector2D is to represent the data of a 2d vector, which is simply x and y. A Transform2D represent a 2D transformation, which not only have some internal data to it, but also other operations associated. 

The design of Transform2D is more of a interface then simple data grouping. Transform2D represent the interface of how Geometry objects could be manipulated by the Transform2D. As well as how Transform2D itself could very.

The internal data of Transform2D have dependencies on each other, which for Vector2D, the internal x and y, are rather independent. In addition, Transform2D could have many different implementation for the data storage. Thus it is necessary to use public/private to draw the distinction between interface and implementation, which makes it fall within the usecase of classes (struct usually shouldn't have private members)   


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

This is to avoid unintended conversion. As c++ allows function overloading, the constructor that have a single argument of Ponit2D and Vector2D could be easily confused, or even interchanged by compiler if caller uses inplace constructor like `{num,num}`. Explicit forces the caller to specify the type if any ambiguity are possible.

In general, single argument constructor should always be explicit.

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

The const function means by calling this function, no member variables (except mutable or pointer) will be modified. This allows for more optimization, better readability, cleaner design intent and prevents error.

The Transform2D::inv() only read the member variable and output the result in return value. Which allows it to be a const function. 

On the other hand, Transform2D::operator*=() is the same as multiply, then assign to self. Thus it certainly modify the object itself and can not be a const function. 