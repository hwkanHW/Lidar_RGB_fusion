#include <iostream>
#include <string>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>


using namespace std;

class directory{
    public:
        directory(string a);
        directory(string a, vector<string> &b);
        string getInputDir();
        void setDir(string a);
        vector<string> getFileNameSet();
        void getFileName(string inputDir);

    private:
        string inputDir;
        vector<string> fileNameSet;
    };

directory::directory(string a)
{
    this->inputDir = a;
}

directory::directory(string a, vector<string> &b)
{
    this->inputDir = a;
    this->fileNameSet = b;
}

string directory::getInputDir()
{
    return this->inputDir;
}

void directory::setDir(string a)
{
    inputDir = a;
}

vector<string> directory::getFileNameSet()
{
    return this->fileNameSet;
}

void directory::getFileName(string inputDir)
{
    if(inputDir.empty())
    {   
        return;
    }
    struct stat statBuf;
    mode_t modes;
    lstat(inputDir.c_str(), &statBuf);
    modes = statBuf.st_mode;
    if(S_ISREG(modes))
    {
        fileNameSet.push_back(inputDir);
        return;
    }
    if(S_ISDIR(modes))
    {
        struct dirent *dir;
        DIR *dp;
        if((dp = opendir(inputDir.c_str())) == NULL)
        {
            cerr<<"open directory error";
            return;
        }
        while((dir = readdir(dp)) != NULL)
        {
            if(strcmp(".", dir->d_name) ==0 || strcmp("..", dir->d_name) == 0)
            {
                continue;
            }
            string subFileName = inputDir + "/" + dir->d_name;
            lstat(subFileName.c_str(), &statBuf);
            if(S_ISREG(statBuf.st_mode))
            {
                fileNameSet.push_back(subFileName);
            }
            if(S_ISDIR(statBuf.st_mode))
            {
                getFileName(subFileName);
            }
        }
        closedir(dp);
    }
}

