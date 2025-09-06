/**
 * Copyright (c) 2024-2025 Sjofn LLC
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TASK_H_
#define TASK_H_

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Arduino.h>

class Task {
public:
    explicit Task(String taskName = "Task", uint16_t stackSize = 10000, uint8_t priority = 5);
    virtual ~Task();
    void setStackSize(uint16_t stackSize);
    void setPriority(uint8_t priority);
    void setName(String const& name);
    void setCore(BaseType_t coreId);
    void start(void* taskData = nullptr);
    void stop();
    /**
     * @brief Body of the task to execute.
     *
     * This function must be implemented in the subclass that represents the actual task to run.
     * When a task is started by calling start(), this is the code that is executed in the
     * newly created task.
     *
     * @param [in] data The data passed in to the newly started task.
     */
    virtual void run(void* data) = 0; // Make run pure virtual
    static void delay(int ms);

private:
    TaskHandle_t m_handle;
    void*        m_taskData;
    static void  runTask(void* pTaskInstance);
    String       m_taskName;
    uint16_t     m_stackSize;
    uint8_t      m_priority;
    BaseType_t   m_coreId;
};

#endif // TASK_H_