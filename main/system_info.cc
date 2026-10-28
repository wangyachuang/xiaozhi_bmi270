#include "system_info.h"

#include <freertos/task.h>
#include <esp_log.h>
#include <esp_flash.h>
#include <esp_mac.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <esp_partition.h>
#include <esp_app_desc.h>
#include <esp_ota_ops.h>
#if CONFIG_IDF_TARGET_ESP32P4
#include "esp_wifi_remote.h"
#endif

#define TAG "SystemInfo"

size_t SystemInfo::GetFlashSize() {
    uint32_t flash_size;
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get flash size");
        return 0;
    }
    return (size_t)flash_size;
}

size_t SystemInfo::GetMinimumFreeHeapSize() {
    return esp_get_minimum_free_heap_size();
}

size_t SystemInfo::GetFreeHeapSize() {
    return esp_get_free_heap_size();
}

std::string SystemInfo::GetMacAddress() {
    uint8_t mac[6];
#if CONFIG_IDF_TARGET_ESP32P4
    esp_wifi_get_mac(WIFI_IF_STA, mac);
#else
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
#endif
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return std::string(mac_str);
}

std::string SystemInfo::GetChipModelName() {
    return std::string(CONFIG_IDF_TARGET);
}

std::string SystemInfo::GetUserAgent() {
    auto app_desc = esp_app_get_description();
    auto user_agent = std::string(BOARD_NAME "/") + app_desc->version;
    return user_agent;
}

esp_err_t SystemInfo::PrintTaskCpuUsage(TickType_t xTicksToWait) {
    #define ARRAY_SIZE_OFFSET 5
    TaskStatus_t *start_array = NULL, *end_array = NULL;
    UBaseType_t start_array_size, end_array_size;
    configRUN_TIME_COUNTER_TYPE start_run_time, end_run_time;
    esp_err_t ret;
    uint32_t total_elapsed_time;

    //Allocate array to store current task states
    start_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    start_array = (TaskStatus_t*)malloc(sizeof(TaskStatus_t) * start_array_size);
    if (start_array == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto exit;
    }
    //Get current task states
    start_array_size = uxTaskGetSystemState(start_array, start_array_size, &start_run_time);
    if (start_array_size == 0) {
        ret = ESP_ERR_INVALID_SIZE;
        goto exit;
    }

    vTaskDelay(xTicksToWait);

    //Allocate array to store tasks states post delay
    end_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    end_array = (TaskStatus_t*)malloc(sizeof(TaskStatus_t) * end_array_size);
    if (end_array == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto exit;
    }
    //Get post delay task states
    end_array_size = uxTaskGetSystemState(end_array, end_array_size, &end_run_time);
    if (end_array_size == 0) {
        ret = ESP_ERR_INVALID_SIZE;
        goto exit;
    }

    //Calculate total_elapsed_time in units of run time stats clock period.
    total_elapsed_time = (end_run_time - start_run_time);
    if (total_elapsed_time == 0) {
        ret = ESP_ERR_INVALID_STATE;
        goto exit;
    }

    printf("| Task | Run Time | Percentage\n");
    //Match each task in start_array to those in the end_array
    for (int i = 0; i < start_array_size; i++) {
        int k = -1;
        for (int j = 0; j < end_array_size; j++) {
            if (start_array[i].xHandle == end_array[j].xHandle) {
                k = j;
                //Mark that task have been matched by overwriting their handles
                start_array[i].xHandle = NULL;
                end_array[j].xHandle = NULL;
                break;
            }
        }
        //Check if matching task found
        if (k >= 0) {
            uint32_t task_elapsed_time = end_array[k].ulRunTimeCounter - start_array[i].ulRunTimeCounter;
            uint32_t percentage_time = (task_elapsed_time * 100UL) / (total_elapsed_time * CONFIG_FREERTOS_NUMBER_OF_CORES);
            printf("| %-16s | %8lu | %4lu%%\n", start_array[i].pcTaskName, task_elapsed_time, percentage_time);
        }
    }

    //Print unmatched tasks
    for (int i = 0; i < start_array_size; i++) {
        if (start_array[i].xHandle != NULL) {
            printf("| %s | Deleted\n", start_array[i].pcTaskName);
        }
    }
    for (int i = 0; i < end_array_size; i++) {
        if (end_array[i].xHandle != NULL) {
            printf("| %s | Created\n", end_array[i].pcTaskName);
        }
    }
    ret = ESP_OK;

exit:    //Common return path
    free(start_array);
    free(end_array);
    return ret;
}

void SystemInfo::PrintTaskList() {
    char buffer[1000];
    vTaskList(buffer);
    ESP_LOGI(TAG, "Task list: \n%s", buffer);
}

void SystemInfo::PrintHeapStats() {
    int free_sram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    int min_free_sram = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    ESP_LOGI(TAG, "free sram: %u minimal sram: %u", free_sram, min_free_sram);
}

void SystemInfo::PrintMemoryUsage() {
    // Internal SRAM (the scarce resource; DMA buffers and task stacks live here)
    const size_t sram_total  = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    const size_t sram_free   = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const size_t sram_min    = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    // Largest contiguous block: tells fragmentation apart from a real leak.
    // A healthy heap keeps this close to "free"; a fragmented one collapses.
    const size_t sram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);

    // Internal DMA-capable memory (I2S/LCD transfers must come from this pool)
    const size_t dma_free  = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    const size_t dma_min   = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);

    // PSRAM (present on ESP32-S3 modules with external RAM)
    const size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    const size_t psram_free  = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    const size_t psram_min   = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);

    const unsigned sram_used_pct = sram_total ? (unsigned)((sram_total - sram_free) * 100 / sram_total) : 0;
    const unsigned sram_frag_pct = sram_free ? (unsigned)(100 - (sram_largest * 100 / sram_free)) : 0;

    ESP_LOGI(TAG, "MEM sram: used=%u%% free=%u/%u min=%u largest=%u frag=%u%% | dma free=%u min=%u",
             sram_used_pct, (unsigned)sram_free, (unsigned)sram_total,
             (unsigned)sram_min, (unsigned)sram_largest, sram_frag_pct,
             (unsigned)dma_free, (unsigned)dma_min);
    if (psram_total > 0) {
        ESP_LOGI(TAG, "MEM psram: free=%u/%u min=%u",
                 (unsigned)psram_free, (unsigned)psram_total, (unsigned)psram_min);
    }
}

void SystemInfo::StartMemoryMonitor(uint32_t interval_ms) {
    static esp_timer_handle_t mem_timer = nullptr;
    if (mem_timer != nullptr) {
        return;  // already running
    }
    esp_timer_create_args_t args = {};
    args.callback = [](void* arg) {
        SystemInfo::PrintMemoryUsage();
    };
    args.arg = nullptr;
    args.dispatch_method = ESP_TIMER_TASK;
    args.name = "mem_mon";
    args.skip_unhandled_events = true;

    if (esp_timer_create(&args, &mem_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create memory monitor timer");
        mem_timer = nullptr;
        return;
    }
    if (esp_timer_start_periodic(mem_timer, (uint64_t)interval_ms * 1000ULL) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start memory monitor timer");
        esp_timer_delete(mem_timer);
        mem_timer = nullptr;
        return;
    }
    ESP_LOGI(TAG, "Memory monitor started, interval=%u ms", (unsigned)interval_ms);
    // Print one sample immediately so the startup baseline is visible.
    SystemInfo::PrintMemoryUsage();
}
