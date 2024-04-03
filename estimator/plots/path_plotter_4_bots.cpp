#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include "gnuplot-iostream.h"
using std::cout;
using std::endl;
using std::vector;

int main() {
    std::ifstream file("../data/saved_data_for_test_1_4_bots_fin.bin", std::ios::binary);
    
    
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
    vector<double> xcr3;
    vector<double> xcr4;
    vector<double> ycr1;
    vector<double> ycr2;
    vector<double> ycr3;
    vector<double> ycr4;
    vector<double> odom1_x;
    vector<double> odom1_y;
    vector<double> odom4_x;
    vector<double> odom4_y;
    vector<double> cam1_x;
    vector<double> cam1_y;
    vector<double> cam4_x;
    vector<double> cam4_y;
    vector<double> acc1_x;
    vector<double> acc1_y;
    vector<double> acc4_x;
    vector<double> acc4_y;
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
            // savedDataR2.push_back(savedData[i]);
        }
        if (savedData[i][1] == 3.0) {
            xcr3.push_back(savedData[i][5]);
            ycr3.push_back(savedData[i][6]);
            // savedDataR3.push_back(savedData[i]);
        }
        if (savedData[i][1] == 4.0) {
            xcr4.push_back(savedData[i][5]);
            ycr4.push_back(savedData[i][6]);
            odom4_x.push_back(savedData[i][2]);
            odom4_y.push_back(savedData[i][3]);
            cam4_x.push_back(savedData[i][8]);
            cam4_y.push_back(savedData[i][9]);
            acc4_x.push_back(savedData[i][21]);
            acc4_y.push_back(savedData[i][22]);
            // savedDataR4.push_back(savedData[i]);
        }

    }
    // cout << "Sample data: Time: " << savedData[0][0] << ", Odom timer:" << savedData[40][14] << ", Cam timer: " << savedData[40][15]-savedData[39][15] << endl;

    Gnuplot gp;
    gp << "set xlabel 'X'" << endl;
    gp << "set ylabel 'Y'" << endl;
    gp << "set title 'Robot trajectory'" << endl;
    gp << "set grid" << endl;
    gp << "plot '-' with linespoints pt 3 lc 'blue' title 'Robot 1',"
        << "'-' with linespoints pt 3 lc 'orange' title 'Robot 2',"
        << "'-' with linespoints pt 3 lc 'magenta' title 'Robot 3',"
        << "'-' with linespoints pt 3 lc 'brown' title 'Robot 4',"
        << "'-' with points pt 7 ps 2 lc 'red' title 'Robot 1 initial position',"
        << "'-' with points pt 7 ps 2 lc 'red' title 'Robot 2 initial position',"
        << "'-' with points pt 7 ps 2 lc 'red' title 'Robot 3 initial position',"
        << "'-' with points pt 7 ps 2 lc 'red' title 'Robot 4 initial position',"
        << "'-' with points pt 5 ps 2 lc 'green' title 'Robot 1 final position',"
        << "'-' with points pt 5 ps 2 lc 'green' title 'Robot 2 final position',"
        << "'-' with points pt 5 ps 2 lc 'green' title 'Robot 3 final position',"
        << "'-' with points pt 5 ps 2 lc 'green' title 'Robot 4 final position'," 
        << "'-' with points pt 2 ps 1.5 lc 'blue' title 'Robot 1 goal',"
        << "'-' with points pt 2 ps 1.5 lc 'orange' title 'Robot 2 goal',"
        << "'-' with points pt 2 ps 1.5 lc 'magenta' title 'Robot 3 goal',"
        << "'-' with points pt 2 ps 1.5 lc 'brown' title 'Robot 4 goal'" << endl;
    gp.send1d(std::make_tuple(xcr1, ycr1));
    gp.send1d(std::make_tuple(xcr2, ycr2));
    gp.send1d(std::make_tuple(xcr3, ycr3));
    gp.send1d(std::make_tuple(xcr4, ycr4));
    gp.send1d(std::make_tuple(vector<double>{-0.586}, vector<double>{-0.416}));
    gp.send1d(std::make_tuple(vector<double>{0.411}, vector<double>{-0.402}));
    gp.send1d(std::make_tuple(vector<double>{0.421}, vector<double>{0.722}));
    gp.send1d(std::make_tuple(vector<double>{-0.559}, vector<double>{0.752}));
    gp.send1d(std::make_tuple(vector<double>{xcr1.back()}, vector<double>{ycr1.back()}));
    gp.send1d(std::make_tuple(vector<double>{xcr2.back()}, vector<double>{ycr2.back()}));
    gp.send1d(std::make_tuple(vector<double>{xcr3.back()}, vector<double>{ycr3.back()}));
    gp.send1d(std::make_tuple(vector<double>{xcr4.back()}, vector<double>{ycr4.back()}));
    gp.send1d(std::make_tuple(vector<double>{-0.079}, vector<double>{-0.286}));
    gp.send1d(std::make_tuple(vector<double>{-0.079}, vector<double>{0.014}));
    gp.send1d(std::make_tuple(vector<double>{-0.079}, vector<double>{0.314}));
    gp.send1d(std::make_tuple(vector<double>{-0.079}, vector<double>{0.614}));

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
    gp1.send1d(std::make_tuple(vector<double>{-0.586}, vector<double>{-0.416}));
    gp1.send1d(std::make_tuple(vector<double>{xcr1.back()}, vector<double>{ycr1.back()}));

    Gnuplot gp2;
    gp2 << "set xlabel 'X'" << endl;
    gp2 << "set ylabel 'Y'" << endl;
    gp2 << "set title 'Robot trajectory'" << endl;
    gp2 << "set grid" << endl;
    gp2 << "plot '-' with linespoints pt 3 lc 'blue' title 'Robot 4 - odometry',"
        << "'-' with linespoints pt 3 lc 'orange' title 'Robot 4 - camera',"
        << "'-' with linespoints pt 3 lc 'magenta' title 'Robot 4 - accelerometer',"
        << "'-' with linespoints pt 3 lc 'brown' title 'Robot 4 - mr-EKF estimate',"
        << "'-' with points pt 7 ps 2 lc 'red' title 'Robot 4 initial position',"
        << "'-' with points pt 5 ps 2 lc 'green' title 'Robot 4 final position'" << endl;
    gp2.send1d(std::make_tuple(odom4_x, odom4_y));
    gp2.send1d(std::make_tuple(cam4_x, cam4_y));
    gp2.send1d(std::make_tuple(acc4_x, acc4_y));
    gp2.send1d(std::make_tuple(xcr4, ycr4));
    gp2.send1d(std::make_tuple(vector<double>{-0.559}, vector<double>{0.752}));
    gp2.send1d(std::make_tuple(vector<double>{xcr4.back()}, vector<double>{ycr4.back()}));


    std::cin.get();
    return 0;
}