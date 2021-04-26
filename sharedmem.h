#ifndef SHAREDMEM_H
#define SHAREDMEM_H

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>

#include <QSharedMemory>
#include <iostream>

#define MAX_DATA_SIZE 1000

//for system V shared mem
#define BUF_SIZE_BYTES 8192
#define BAD_RESULT -1

//#define SHARED_MEMORY_READER 0444
#define SHARED_MEMORY_WRITER 0666

#define SHM_KEY 0x1
#define FILE_NAME "/home/devel/Projects/MainKinematic/ForKeys.txt"
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


struct shmseg
{
    int cnt;
    int complete;
    float buf[BUF_SIZE_BYTES / sizeof(float)];
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
class SharedMemLinux
{
public:
    //Shared memory class based on mmap
    SharedMemLinux(char const *shm_block_name, const size_t block_size_bytes)
    {
//        m_shmid = shmget(m_key, m_size, IPC_CREAT);         //creating shared memory segment
//        if (m_shmid == -1)
//            perror("error creating shared memory");
//        else{
//            m_shmptr = (shmseg*)shmat(m_shmid, NULL, 0);    //attaching shared memory
//            if (m_shmptr == (void *) -1)
//                perror("error attaching shared memory");
//            m_buf_ptr = m_shmptr->buf;                      //get pointer to shared memory buffer
//        }
//        m_sem_ptr = semget()

        m_size = block_size_bytes;
//        m_data = new float [m_size];
//        if ((m_key = ftok(shm_block_name, 0)) == -1)
//            perror("");
//        m_key = SHM_KEY;
        m_key = ftok(shm_block_name, 0);
        if ((m_shmid = shmget(m_key, block_size_bytes, IPC_CREAT | SHARED_MEMORY_WRITER)) == -1)
            perror("Error shmget");
        if ((m_sem_id = semget(m_key, 1, IPC_CREAT | SHARED_MEMORY_WRITER)) == -1)
            perror("Error getting semaphore");
        if (m_shmid == BAD_RESULT)
            perror("Error getting block ID");
        m_shmptr = (shmseg*)shmat(m_shmid, NULL, 0);
        if (m_shmptr == (void*)BAD_RESULT)
            perror("Error attaching shared memory");
    }
    bool SendData(const float *data, const size_t size_elements)
    {
        if (size_elements > BUF_SIZE_BYTES / sizeof(float))
            return -1;
        printf("Trying to lock semaphore\n");
//        if (semop(m_sem_id, &semb, 1) == -1)
//            perror("Error semop lock");
        if (semctl(m_sem_id, 0, SETVAL, 0) == -1)
            perror("Errro SETVAL");
        else
        {
            printf("locked\n");
            if (size_elements * sizeof(float) > m_size)
                printf("Cant send, array size more than possible");
            else
                memcpy(m_shmptr->buf, data, size_elements * sizeof(float));
        }
//        if (semop(m_sem_id, &semb, 1) == -1)
//            perror("Error semop unlock");
        if (semctl(m_sem_id, 0, SETVAL, 1) == -1)
            perror("Errro SETVAL");
        else
            printf("Semaphore unlocked\n");
        return 0;
    }
    bool ReadData(const size_t size_elements)
    {
        if (m_shmptr == NULL)
        {
            perror("Shared memory is not attached");
            return -1;
        }
//        if (semop(m_sem_id, &semb, 1) == -1)
//            perror("Error semop lock");
        if (semctl(m_sem_id, 0, SETVAL, 0) == -1)
            perror("Errro SETVAL");
        else
        {
            printf("Semaphore locked\n");
//        else
//        {
//            printf("locked\n");
            if (size_elements * sizeof(float) > m_size)
                printf("Cant send, array size more than possible");
            else
                memcpy(m_data, m_shmptr->buf, size_elements * sizeof(float));
        }
//        if (semop(m_sem_id, &semb, 1) == -1)
//            perror("Error semop unlock");
        if (semctl(m_sem_id, 0, SETVAL, 1) == -1)
            perror("Errro SETVAL");
        else
            printf("Semaphore unlocked\n");
        return 0;
    }
    size_t GetSize() const
    {
        return m_size;
    };
    ~SharedMemLinux()
    {
        //detach
        if (shmdt(m_shmptr) == BAD_RESULT)
            perror("Error detaching");
        //destroy

    }
private:
    key_t m_key;

    size_t m_size;
    int m_shmid;
    shmseg *m_shmptr;
    int m_sem_id;
    float *m_data;
    sembuf semb = {0, 1, 0};
//float *m_buf_ptr;
//    sem_t *m_sem_ptr;

};

#endif // SHAREDMEM_H
