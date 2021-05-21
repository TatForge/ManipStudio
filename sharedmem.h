#ifndef SHAREDMEM_H
#define SHAREDMEM_H


#include <QSharedMemory>
#include <iostream>

#define MAX_DATA_SIZE 1000

class SharedMem
{
public:


    SharedMem(QString key, size_t size = 10) : m_key(key), m_size(size)
    {//size = elements count
        sh_m = new QSharedMemory(m_key);
        m_data = new float [m_size];
        if (sh_m->isAttached())
        {
            sh_m->detach();
        }
        if (!sh_m->create(sizeof(float) * m_size))
        {
            std::cout << sh_m->errorString().toStdString() << std::endl;
        }
        if (!sh_m->attach())
        {
            std::cout << "can't attach" << std::endl;
        }
    };
    ~SharedMem()
    {
        delete [] m_data;
        if (sh_m->detach())
            std::cout << "Shared memory " << m_key.toStdString() << " detached " << std::endl;
    }
    bool SendData(float *data_ptr, size_t size)
    {
        //sends data from data_ptr, size = m_size
        if (sh_m->lock())
        {
//            auto ptr = sh_m->data();
            memcpy(sh_m->data(), data_ptr, size);
            sh_m->unlock();
        } else {
            return false;
        }
        return true;
    }
    size_t LoadDataSize()
    {
        m_size = (*(size_t*)sh_m->data());
        if (m_size > MAX_DATA_SIZE)
        {
            throw(QString("Size is larger than the max possible"));
        }
        return m_size;
    }
    size_t GetSize() const
    {
        return m_size;
    };
    void* GetData()
    {
        sh_m->lock();
        memcpy(m_data, sh_m->data(), sizeof(float) * m_size);
        sh_m->unlock();
        return m_data;
    };
private:
    QSharedMemory *sh_m;
    QString m_key;
    size_t m_size;
    float *m_data;

};



//struct sembuf
//{
//    pid_t sempid;
//    ushort semval;
//    ushort semncnt;
//    ushort semzcnt;
//};

//enum SHMType
//{
//    Reader,
//    Writer
//};

#endif // SHAREDMEM_H
