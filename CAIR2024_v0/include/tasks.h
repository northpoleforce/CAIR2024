#ifndef TASKS_H
#define TASKS_H

struct Task
{
    bool have, arrived, ready, putDown, done;
};

extern Task task[6];

#endif