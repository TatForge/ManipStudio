#ifndef SYSVSHAREDMEMORY_H
#define SYSVSHAREDMEMORY_H

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>

#include <iostream>
#include <cstring>
#include <mutex>


//for system V shared mem
#define BUF_SIZE_BYTES 8192
#define BAD_RESULT -1

//#define SHARED_MEMORY_READER 0444
#define SHARED_MEMORY_WRITER 0666

#define FILE_POINTS_SHM "/home/devel/Projects/MainKinematic/SharedMemoryFiles/Points.txt"
#define FILE_MANIP_SHM "/home/devel/Projects/MainKinematic/SharedMemoryFiles/Manip.txt"
#define FILE_MODELS_SHM "/home/devel/Projects/MainKinematic/SharedMemoryFiles/3dModels.txt"

struct shmseg
{
    int cnt;
    int complete;
    float buf[BUF_SIZE_BYTES / sizeof(float)];
};

class SysVSharedMemory
{
public:
    //Shared memory class based on mmap
    SysVSharedMemory(char const *shm_block_name, const size_t block_size_bytes)
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
        m_data = new float [m_size / sizeof(float)];
//        if ((m_key = ftok(shm_block_name, 0)) == -1)
//            perror("");
//        m_key = SHM_KEY;
        if ((m_key = ftok(shm_block_name, 0)) == -1)
            perror("Error getting key from ftok");
//        m_key = 0x103222;

        if ((m_shmid = shmget(m_key, block_size_bytes, IPC_CREAT | SHARED_MEMORY_WRITER)) == -1)
            perror("Error shmget");
        if ((m_sem_id = semget(m_key, 1, IPC_CREAT | IPC_EXCL | SHARED_MEMORY_WRITER)) == -1)
        {
            perror("The Semaphore is already open");
            //Taking semaphore ID if semaphore exists
            if ((m_sem_id = semget(m_key, 1, IPC_CREAT | SHARED_MEMORY_WRITER)) == -1)
                perror("Error getting semaphore");
        }
        else
        {
            //Create new semaphore and set value 1
            if ((m_sem_id = semget(m_key, 1, IPC_CREAT | SHARED_MEMORY_WRITER)) == -1)
                perror("Error getting semaphore");
            else
                semctl(m_sem_id, 0, SETVAL, 1);
        }


        auto ii = semctl(m_sem_id, 0, GETVAL);
//        if ((semctl(m_sem_id, 0, SETVAL, 1)) == -1)
//            perror("Error Sem SETVAL");
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
//        if (semctl(m_sem_id, 0, SETVAL, 0) == -1)
//            perror("Errro SETVAL");
        if ((semop(m_sem_id, &sem_lock, 1)) == -1)
            perror("Error semop lock");
        else
        {
            printf("locked\n");
            if (size_elements * sizeof(float) > m_size)
                printf("Cant send, array size more than possible");
            else
                memcpy(m_shmptr->buf, data, size_elements * sizeof(float));
            if ((semop(m_sem_id, &sem_unlock, 1)) == -1)
                perror("Error semop unlock");
        }
//        if (semop(m_sem_id, &semb, 1) == -1)
//            perror("Error semop unlock");
//        if (semctl(m_sem_id, 0, SETVAL, 1) == -1)
//            perror("Errro SETVAL");
//        printf("Semaphore unlocked\n");
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
//        if (semctl(m_sem_id, 0, SETVAL, 0) == -1)
//            perror("Errro SETVAL");
        auto ii = semctl(m_sem_id, 0, GETVAL);
        if ((semop(m_sem_id, &sem_lock, 1)) == -1)
        {
            perror("Error semop lock");
            auto kk = errno;
            auto kkl = semctl(m_sem_id, 0, GETVAL);
            auto kk2 = 1;
        }
        else
        {
            printf("Semaphore locked\n");
            if (size_elements * sizeof(float) > m_size)
                printf("Cant send, array size more than possible");
            else
                memcpy(m_data, m_shmptr->buf, size_elements * sizeof(float));
            if ((semop(m_sem_id, &sem_unlock, 1)) == -1)
                perror("Error semop unlock");
            else
                printf("Semaphore unlocked\n");
        }

        auto ii2 = semctl(m_sem_id, 0, GETVAL);
//        if (semop(m_sem_id, &semb, 1) == -1)
//            perror("Error semop unlock");
//        if (semctl(m_sem_id, 0, SETVAL, 1) == -1)
//            perror("Errro SETVAL");
//        else
//            printf("Semaphore unlocked\n");
        return 0;
    }
    size_t GetSize() const
    {
        return m_size;
    };
    ~SysVSharedMemory()
    {
        //detach
        if (shmdt(m_shmptr) == -1)
            perror("Error detaching");
        //destroy
        if (shmctl(m_shmid, IPC_RMID, 0) == -1)
            perror("Error deleting shared memory");
        if (semctl(m_sem_id, 0, IPC_RMID, 0) == -1)
            perror("Error deleting semaphore");
    }
private:
    key_t m_key;
    size_t m_size;
    int m_shmid;
    shmseg *m_shmptr;
    int m_sem_id;
    float *m_data;
    std::mutex m_shm_mtx;
    sembuf sem_lock = {0, -1, 0};
    sembuf sem_unlock = {0, 1, 0};

//    sembuf semb = {0, 1, 0};
//    shmid_ds = {0777, 1};
//float *m_buf_ptr;
//    sem_t *m_sem_ptr;

};

#endif // SYSVSHAREDMEMORY_H
