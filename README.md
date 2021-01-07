#### Arduino: Sterownik dmuchawy oraz 3 klap
  
Projekt tworzony i usprawniany w latach 2019 - 2020.
  
Celem było zaprojektowanie i wykonanie sterownika dmuchawy oraz 3 klap z wysoką kulturą pracy.  
Zmiana RPM dmuchawy oraz położenia klap ma zachodzić w sposób jak najmniej zwracający na siebie uwagę, płynnie.  
Zadane wartości można ustawiać poprzez szynę I2C.  
  
Do wysterowania dmuchawy zasilanej napięciem 230VAC wykorzystałem [moduł AVT5664](https://sklep.avt.pl/avt5664.html), wykorzystując sterowanie fazowe dla obciążeń indukcyjnych. Można skorzystać z dwóch szybkości zmiany RPM dmuchawy.
  
W celu zrealizowania projektu wykorzystałem bezpośrednio biblioteki AVR (optymalizacja wydajnościowa).

