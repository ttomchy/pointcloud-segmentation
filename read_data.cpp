#include <iostream>
#include <fstream>
using namespace std;

void readTxt(string file)
{
    ifstream infile;
    infile.open(file.data());   //将文件流对象与文件连接起来
   // assert(infile.is_open());   //若失败,则输出错误消息,并终止程序运行

    string s;
    while(getline(infile,s))
    {
        cout<<s<<endl;
    }
    infile.close();             //关闭文件输入流
}


int main(){

    readTxt("./dataset/train/result.txt");
    std::cout<< "ending !!!"<<std::endl;

}

