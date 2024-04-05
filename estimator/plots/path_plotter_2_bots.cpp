// To run this file, run in terminal: g++ -std=c++17 -I {path to gnuplot-iostream folder} path_plotter_2_bots.cpp -o path2

#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include "gnuplot-iostream.h"
using std::cout;
using std::endl;
using std::vector;

int main() {
    std::ifstream file("../data/saved_data_for_test_2_bots.bin", std::ios::binary);
    
    
    vector<vector<double>> savedData;
    if (file.is_open()) {
        while(!file.eof()) {
            size_t row_size;
            file.read(reinterpret_cast<char*>(&row_size), sizeof(row_size));
            if (file) {
                std::vector<double> row;
                row.resize(row_size);
                file.read(reinterpret_cast<char*>(row.data()), row_size*sizeof(double));
                if (file) {
                    savedData.push_back(row);
                } else {
                    std::cerr << "Cannot read data from file" << endl;
                    break;
                }
            } else {
                if (file.eof()) {
                    file.clear();
                } else {
                    std::cerr << "Cannot read size from file" << endl;
                }
                break;
            }
        }
        file.close();
        cout << "All data has been retireved" << endl;
    } else {
        std::cerr << "File cannot be opened" << endl;
    }
    // vector<vector<double>> savedDataR1;
    // vector<vector<double>> savedDataR2;
    // vector<vector<double>> savedDataR3;
    // vector<vector<double>> savedDataR4;
    vector<double> xcr1;
    vector<double> xcr2;
    vector<double> ycr1;
    vector<double> ycr2;

    vector<double> odom1_x;
    vector<double> odom1_y;
    vector<double> odom2_x;
    vector<double> odom2_y;
    vector<double> cam1_x;
    vector<double> cam1_y;
    vector<double> cam2_x;
    vector<double> cam2_y;
    vector<double> acc1_x;
    vector<double> acc1_y;
    vector<double> acc2_x;
    vector<double> acc2_y;
    for (int i = 0; i < savedData.size(); i++) {
        if (savedData[i][1] == 1.0) {
            xcr1.push_back(savedData[i][5]);
            ycr1.push_back(savedData[i][6]);
            odom1_x.push_back(savedData[i][2]);
            odom1_y.push_back(savedData[i][3]);
            cam1_x.push_back(savedData[i][8]);
            cam1_y.push_back(savedData[i][9]);
            acc1_x.push_back(savedData[i][21]);
            acc1_y.push_back(savedData[i][22]);
            // savedDataR1.push_back(savedData[i]);
        }
        if (savedData[i][1] == 2.0) {
            xcr2.push_back(savedData[i][5]);
            ycr2.push_back(savedData[i][6]);
            odom2_x.push_back(savedData[i][2]);
            odom2_y.push_back(savedData[i][3]);
            cam2_x.push_back(savedData[i][8]);
            cam2_y.push_back(savedData[i][9]);
            acc2_x.push_back(savedData[i][21]);
            acc2_y.push_back(savedData[i][22]);
            // savedDataR2.push_back(savedData[i]);
        }
    }

    Gnuplot gp;
    gp << "set xlabel 'X'" << endl;
    gp << "set ylabel 'Y'" << endl;
    gp << "set title 'Robot trajectory'" << endl;
    gp << "set grid" << endl;
    gp << "plot '-' with linespoints pt 3 lc 'blue' title 'Robot 1',"
        << "'-' with linespoints pt 3 lc 'orange' title 'Robot 2',"
        << "'-' with points pt 7 ps 2 lc 'red' title 'Robot 1 initial position',"
        << "'-' with points pt 7 ps 2 lc 'red' title 'Robot 2 initial position',"
        << "'-' with points pt 5 ps 2 lc 'green' title 'Robot 1 final position',"
        << "'-' with points pt 5 ps 2 lc 'green' title 'Robot 2 final position'" << endl;
    gp.send1d(std::make_tuple(xcr1, ycr1));
    gp.send1d(std::make_tuple(xcr2, ycr2));
    gp.send1d(std::make_tuple(vector<double>{xcr1.front()}, vector<double>{ycr1.front()}));
    gp.send1d(std::make_tuple(vector<double>{xcr2.front()}, vector<double>{ycr2.front()}));
    gp.send1d(std::make_tuple(vector<double>{xcr1.back()}, vector<double>{ycr1.back()}));
    gp.send1d(std::make_tuple(vector<double>{xcr2.back()}, vector<double>{ycr2.back()}));
    
    Gnuplot gp1;
    gp1 << "set xlabel 'X'" << endl;
    gp1 << "set ylabel 'Y'" << endl;
    gp1 << "set title 'Robot trajectory'" << endl;
    gp1 << "set grid" << endl;
    gp1 << "plot '-' with linespoints pt 3 lc 'blue' title 'Robot 1 - odometry',"
        << "'-' with linespoints pt 3 lc 'orange' title 'Robot 1 - camera',"
        << "'-' with linespoints pt 3 lc 'magenta' title 'Robot 1 - accelerometer',"
        << "'-' with linespoints pt 3 lc 'brown' title 'Robot 1 - mr-EKF estimate',"
        << "'-' with points pt 7 ps 2 lc 'red' title 'Robot 1 initial position',"
        << "'-' with points pt 5 ps 2 lc 'green' title 'Robot 1 final position'" << endl;
    gp1.send1d(std::make_tuple(odom1_x, odom1_y));
    gp1.send1d(std::make_tuple(cam1_x, cam1_y));
    gp1.send1d(std::make_tuple(acc1_x, acc1_y));
    gp1.send1d(std::make_tuple(xcr1, ycr1));
    gp1.send1d(std::make_tuple(vector<double>{xcr1.front()}, vector<double>{ycr1.front()}));
    gp1.send1d(std::make_tuple(vector<double>{xcr1.back()}, vector<double>{ycr1.back()}));

    Gnuplot gp2;
    gp2 << "set xlabel 'X'" << endl;
    gp2 << "set ylabel 'Y'" << endl;
    gp2 << "set title 'Robot trajectory'" << endl;
    gp2 << "set grid" << endl;
    gp2 << "plot '-' with linespoints pt 3 lc 'blue' title 'Robot 2 - odometry',"
        << "'-' with linespoints pt 3 lc 'orange' title 'Robot 2 - camera',"
        << "'-' with linespoints pt 3 lc 'magenta' title 'Robot 2 - accelerometer',"
        << "'-' with linespoints pt 3 lc 'brown' title 'Robot 2 - mr-EKF estimate',"
        << "'-' with points pt 7 ps 2 lc 'red' title 'Robot 2 initial position',"
        << "'-' with points pt 5 ps 2 lc 'green' title 'Robot 2 final position'" << endl;
    gp2.send1d(std::make_tuple(odom2_x, odom2_y));
    gp2.send1d(std::make_tuple(cam2_x, cam2_y));
    gp2.send1d(std::make_tuple(acc2_x, acc2_y));
    gp2.send1d(std::make_tuple(xcr2, ycr2));
    gp2.send1d(std::make_tuple(vector<double>{xcr2.front()}, vector<double>{ycr2.front()}));
    gp2.send1d(std::make_tuple(vector<double>{xcr2.back()}, vector<double>{ycr2.back()}));


    std::cin.get();
    return 0;
}