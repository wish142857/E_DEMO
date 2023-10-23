#ifndef EVENT_QUEUE_HPP
#define EVENT_QUEUE_HPP
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

namespace e_demo {

    /**********************
     * @struct EventQueue
     * @brief 1. chronological order 2. save latest events 3. keep max length
     **********************/
    struct  EventQueue {
        int len, num, tail;  // max length / current number / tail pointer
        std::vector<dvs_msgs::Event> que;

        // @brief  [EventQueue]
        // @param {int} length: max length of queue
        EventQueue(const int length) : len(length), num(0), tail(0) { }

        // @brief  [insert] insert event, keep order and max length
        // head: oldest  tail: newest
        void insert(const dvs_msgs::Event& e) {
            if (num < len) {
                // --- insert ---
                tail = num++, que.push_back(e);
                // --- sort ---
                int pre = tail - 1;
                while (pre >= 0 && que[pre].ts > e.ts) {
                    que[pre + 1] = que[pre];
                    pre--;
                }
                que[pre + 1] = e;
            }
            else {
                // --- check ---
                int head = (tail + 1) % len;
                if (e.ts < que[head].ts)
                    return;
                // --- insert ---
                tail = (tail + 1) % len, que[tail] = e;
                // --- sort ---
                int pre = (tail + len - 1) % len;
                while (pre != tail && que[pre].ts > e.ts) {
                    que[(pre + 1) % len] = que[pre];
                    pre = (pre + len - 1) % len;
                }
                que[(pre + 1) % len] = e;
            }   
        }
        
        // @brief  [recent] get recent event before time t
        dvs_msgs::Event* recent(const ros::Time& t) {
            for (int n = 0; n < num; n++) {
                int i = (tail + len - n) % len;
                if (que[i].ts < t)
                    return &que[i];
            }
            return nullptr;
        }
    };


    /**************************
     * @struct EventQueueMat
     **************************/
    struct EventQueueMat {
        int w, h, len;  // width / height / length
        std::vector<std::vector<EventQueue>> mat;

        // @brief  [EventQueueMat]
        // @param {int} length: max length of queue
        EventQueueMat(const int width, const int height, const int length) : w(width), h(height), len(length) {
            mat.resize(w, std::vector<EventQueue>(h, EventQueue(len)));
        }

        // @brief  [insert] insert event, maintain upper limit of length
        void insert(const dvs_msgs::Event& e) {
                if (isInside(e.x, e.y))
                    mat[e.x][e.y].insert(e);
        }

        // @brief  [recent] get recent event before time t
        dvs_msgs::Event* recent(const int x, const int y, const ros::Time& t) {
            if (isInside(x, y))
                return mat[x][y].recent(t);
            return nullptr;
        }

        // @brief  [isInside] check whether the parameters are legal
        inline bool isInside(const int x, const int y) {
            return x >= 0 && x < w && y >= 0 && y < h;
        }
    };
    
}

#endif
