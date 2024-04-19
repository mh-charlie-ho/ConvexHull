#include <fstream>
#include <string>
#include <iostream>
using namespace std;

int Writetxt(float x, float y, string filename="output.txt")
{
    ofstream ofs(filename, ios::app);

    if (!ofs.is_open()) {
        cout << "Failed to open file.\n";
        return 1; // EXIT_FAILURE
    }

    ofs << x << " " << y << "\n";
    ofs.close();
    return 0;

}