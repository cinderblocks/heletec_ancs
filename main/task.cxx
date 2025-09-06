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

#include "Task.h"
#include "sdkconfig.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* LOG_TAG = "Task";

/**
 * @brief Create an instance of the task class.
 *
 * @param [in] taskName The name of the task to create.
 * @param [in] stackSize The size of the stack.
 * @param [in] priority The priority level of this task.
 * @return N/A.
 */
Task::Task(String taskName, uint16_t stackSize, uint8_t priority)
:	m_handle(nullptr)
,	m_taskData(nullptr)
,	m_taskName(std::move(taskName))
,	m_stackSize(stackSize)
,	m_priority(priority)
,	m_coreId(tskNO_AFFINITY)
{ }

/* virtual */
Task::~Task() = default;

/**
 * @brief Suspend the task for the specified milliseconds.
 *
 * @param [in] ms The delay time in milliseconds.
 * @return N/A.
 */

/* static */
void Task::delay(int ms)
{
	::vTaskDelay(ms / portTICK_PERIOD_MS);
}

/**
 * Static class member that actually runs the target task.
 *
 * The code here will run on the task thread.
 * @param [in] pTaskInstance The task to run.
 */
void Task::runTask(void* pTaskInstance)
{
	Task* pTask = static_cast<Task*>(pTaskInstance);
	ESP_LOGD(LOG_TAG, ">> runTask: taskName=%s", pTask->m_taskName.c_str());
	pTask->run(pTask->m_taskData);
	ESP_LOGD(LOG_TAG, "<< runTask: taskName=%s", pTask->m_taskName.c_str());
	pTask->stop();
}

/**
 * @brief Start an instance of the task.
 *
 * @param [in] taskData Data to be passed into the task.
 * @return N/A.
 */
void Task::start(void* taskData)
{
	if (m_handle != nullptr)
	{
		ESP_LOGW(LOG_TAG, "Task::start - There might be a task already running!");
	}
	m_taskData = taskData;
	::xTaskCreatePinnedToCore(&runTask, m_taskName.c_str(),
		m_stackSize, this, m_priority, &m_handle, m_coreId);
}

/**
 * @brief Stop the task.
 *
 * @return N/A.
 */
void Task::stop()
{
	if (m_handle == nullptr) { return; }
	TaskHandle_t temp = m_handle;
	m_handle = nullptr;
	::vTaskDelete(temp);
}

/**
 * @brief Set the stack size of the task.
 *
 * @param [in] stackSize The size of the stack for the task.
 * @return N/A.
 */
void Task::setStackSize(uint16_t stackSize)
{
	m_stackSize = stackSize;
}

/**
 * @brief Set the priority of the task.
 *
 * @param [in] priority The priority for the task.
 * @return N/A.
 */
void Task::setPriority(uint8_t priority)
{
	m_priority = priority;
}

/**
 * @brief Set the name of the task.
 *
 * @param [in] name The name for the task.
 * @return N/A.
 */
void Task::setName(String const& name)
{
	m_taskName = name;
}

/**
 * @brief Set the core number the task has to be executed on.
 * If the core number is not set, tskNO_AFFINITY will be used
 *
 * @param [in] coreId The id of the core.
 * @return N/A.
 */
void Task::setCore(BaseType_t coreId)
{
	m_coreId = coreId;
}
