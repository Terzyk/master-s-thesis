#include "ros/ros.h"
#include "pracownia_badawcza/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dd");
//   if (argc != 3)
//   {
//     ROS_INFO("usage: add_two_ints_client X Y");
//     return 1;
//   }

  // ros::NodeHandle n;
  // ros::ServiceClient client = n.serviceClient<pracownia_badawcza::AddTwoInts>("add_two_ints");
  // pracownia_badawcza::AddTwoInts srv;
  // srv.request.a = 5;
  // srv.request.b = 3;
  // if (client.call(srv))
  // {
  //   ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  // }
  // else
  // {
  //   ROS_ERROR("Failed to call service add_two_ints");
  //   return 1;
  // }

  return 0;
}












// #include <iostream>

// using namespace std;


// void my_function(int x)
// {       
//     int  *ip; 

//     ip = &x;
//     cout << "Value of var variable: ";
//     cout << x << endl;
//     cout << &x << endl;
//     // print the address stored in ip pointer variable
//     cout << "Address stored in ip variable: ";
//     cout << ip << endl;

//     // access the value at the address available in pointer
//     cout << "Value of *ip variable: ";
//     cout << *ip << endl;
// }


// int main () {
//    int  var = 20;   // actual variable declaration.
//    //int  *ip;        // pointer variable 

//    //ip = &var;       // store address of var in pointer variable

// //    cout << "Value of var variable: ";
// //    cout << var << endl;
// //    cout << &var << endl;
// //    // print the address stored in ip pointer variable
// //    cout << "Address stored in ip variable: ";
// //    cout << ip << endl;

// //    // access the value at the address available in pointer
// //    cout << "Value of *ip variable: ";
// //    cout << *ip << endl;

//    my_function(var);



//    return 0;
// }