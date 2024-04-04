#include <fstream>
#include <iostream>
#include <string>
// #include <map>
// #include <boost/archive/text_iarchive.hpp>
#include <vector>
using std::cout;
using std::endl;

int main() {
    std::ifstream file("../data/saved_data_for_test_1.bin", std::ios::binary);
    
    
    std::vector<std::vector<double>> savedData;
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
        std::cerr << "FIle cannot be opened" << endl;
    }
    cout << "Sample data: Time: " << savedData[0][0] << ", Robot ID:" << savedData[0][1] << ", Odom_x: " << savedData[0][2] << endl;
}

// int main() {
//     std::ifstream file("../data/saved_data_for_test_1.bin", std::ios::binary);
//     std::vector<double[23]> savedData;
//     if (file.is_open()) {
//         // std::vector<std::vector<double[23]>> savedData;
//         double t = 0.0;
//         while(!file.eof()) {
//             size_t size;
//             file.read(reinterpret_cast<char*>(&size), sizeof(size));
//             if (file) {
//                 double dataInst[23];
//                 file.read(reinterpret_cast<char*>(&dataInst),size*sizeof(double));
//                 if (file) {
//                     savedData.push_back(dataInst);
//                 } else {
//                     std::cerr << "Cannot read data from file" << endl;
//                     break;
//                 }
//             } else {
//                 if (file.eof()) {
//                     file.clear();
//                 } else {
//                     std::cerr << "Cannot read size from file" << endl;
//                 }
//                 break;
//             }
//         }
        
//         file.close();
//         cout << "Data has been retrieved";
//     } else {
//         std::cerr << "File cannot be opened" << endl;
//     }
    
//     cout << "Sample data: " << savedData[0][2];
// }


// int main() {
//     // cout << "Hello" <<endl;
//     std::ifstream file("../data/saved_data_for_test_1.bin", std::ios::binary);
//     // cout<<"Did i reach?" <<endl;
//     int t = 0;

//     std::map<int, std::map<std::string, std::map<std::string, double>>> savedData;
//     if (file.is_open()) {
//         // cout << "Hiii" <<endl;
//         while (!file.eof()) {
//             // cout<<"Hi hi"<<endl;
//             std::map<std::string, std::map<std::string, double>> dataEntry;
//             file.read(reinterpret_cast<char*>(&dataEntry), sizeof(dataEntry));
//             // cout<<"hmm"<<endl;
//             if (!file.fail()) {
//                 cout<<"before insert"<<endl;
//                 // savedData[t] = std::map<std::string, std::map<std::string, double>>();
//                 // savedData[t].insert(dataEntry.begin(),dataEntry.end());
//                 // for (const auto& dataNode: dataEntry) {
//                 //     cout<<"before print"<<endl;
//                 //     cout<<"Address: "<<dataNode.first<<endl;
//                 //     for (const auto& dataSubNode: dataNode.second) {
//                 //         savedData[t][dataNode.first][dataSubNode.first] = dataSubNode.second;
//                 //     }
//                 // }
//                 savedData[t] = dataEntry;
//                 t++;
//                 cout<<"aft insert"<<endl;

//             } else {
//                 std::cerr << "File read failed" << endl;
//             }
//             std::cout << "AM i here" << std::endl;
//             t += 3;
//         }
//         file.close();
//         double val = 0;
//         val = savedData[0]["1"]["pos_x"];
//         // for (const auto& dataPoint : savedData[5]) {
//         //     std::cout<< "Address: " << dataPoint.first << std::endl;
//         // }
//         std::cout << "Data: " << val << std::endl;
//     } else {
//         std::cerr << "File not opening" << std::endl;
//     }
    
// }