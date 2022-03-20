#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

int main()
{
    std::ifstream palfile("seg_rgbs.txt");
    std::string str = "2    [123, 45, 78]";
    // std::vector<int> vect;

    std::string line;

    while(std::getline(palfile, line))
    {
        std::stringstream ss(line);
	std::vector<int> vect;
        for (int i; ss >> i;) {
            vect.push_back(i);
            
            char nxt = ss.peek();
            while ((nxt == ',' || nxt == ' ' || nxt == '[' || nxt == ']' || nxt == '\t') && !(nxt == '\n')) {
                ss.ignore();
                nxt = ss.peek();
            }
        }

        for (std::size_t i = 0; i < vect.size(); i++)
            std::cout << vect[i] << " ";
        std::cout << std::endl;
    }
    
    return 0;
}
