# master-s-thesis
Praca magisterska związana jest z tematem:
Globalne planowanie ruchu robota z wykorzystaniem lokalnego planera neuronowego dla samochodu kinematycznego.

Krótki opis pliku:
Plik main: map_node.cpp 
W tym pliku odczytywana jest pozycja startowa i końcowa, wywoływany jest planer, a następnie ścieżka przekazywana jest do funkcji odpowiadającej za ruch robota.
Planner zawarty jest w pliku ompl_lib.cpp
points.cpp odpowiada za publikowanie pozycji startowej i końcowej

Odpalenie programu:
1 terminal -> roslaunch pracownia_badawcza planner.launch
2 terminal -> rosrun pracownia_badawcza points
