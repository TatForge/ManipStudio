#ifndef SHAREDMEM_H
#define SHAREDMEM_H


#include <QSharedMemory>
#include <algorithm>
#include <iostream>

#define MAX_DATA_SIZE 1000

class SharedMem
{
public:


    SharedMem(QString key, size_t size = 10) : m_key(key), m_size(size), m_bytes(sizeof(float) * size)
    {//size = elements count
        sh_m = new QSharedMemory(m_key);
        m_data = new float [m_size];
        if (sh_m->isAttached())
        {
            sh_m->detach();
        }
        //create() attaches on success; otherwise the segment may already exist
        if (!sh_m->create(sizeof(float) * m_size) && !sh_m->attach())
        {
            std::cout << "can't create or attach shared memory " << m_key.toStdString() << ": "
                      << sh_m->errorString().toStdString() << std::endl;
        }
    };
    ~SharedMem()
    {
        delete [] m_data;
        if (sh_m->detach())
            std::cout << "Shared memory " << m_key.toStdString() << " detached " << std::endl;
        delete sh_m;
    }
    bool SendData(float *data_ptr, size_t size)
    {
        //sends `size` bytes from data_ptr; the limit is the requested segment size:
        //on Windows QSharedMemory::size() is rounded up to whole pages
        if (!sh_m->isAttached() || size > std::min(m_bytes, size_t(sh_m->size())))
            return false;
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
    size_t m_bytes;     //requested segment size
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
