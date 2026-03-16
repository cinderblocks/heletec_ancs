/**
 * test_notification_def.cxx — Unity tests for the notification_def struct.
 *
 * notification_def is a plain POD struct in notificationservice.h.  No
 * FreeRTOS tasks, queues, or NimBLE code is compiled here — only the struct
 * definition and its inline methods (reset(), isCall(), ATTR_* constants).
 *
 * Stubs in test/stubs/ satisfy the transitive headers:
 *   notificationservice.h → applist.h → freertos/semphr.h (stub)
 *                         → task.h    → freertos/task.h   (stub)
 *                         → freertos/queue.h              (stub)
 */

// Forward-declare NimBLERemoteCharacteristic so notificationservice.h compiles
// without pulling in all of NimBLE (it only uses it as a pointer type).
class NimBLERemoteCharacteristic;
class NimBLEConnInfo {};

#include "unity.h"
#include "notificationservice.h"  // notification_def, NotificationService

#include <cstring>
#include <ctime>

void setUp(void)    {}
void tearDown(void) {}

// ─────────────────────────────────────────────────────────────────────────
// notification_def — default values
// ─────────────────────────────────────────────────────────────────────────

void test_default_title_empty(void)
{
    notification_def n;
    TEST_ASSERT_EQUAL_UINT8('\0', n.title[0]);
}

void test_default_message_empty(void)
{
    notification_def n;
    TEST_ASSERT_EQUAL_UINT8('\0', n.message[0]);
}

void test_default_bundle_id_empty(void)
{
    notification_def n;
    TEST_ASSERT_EQUAL_UINT8('\0', n.bundleId[0]);
}

void test_default_showed_false(void)
{
    notification_def n;
    TEST_ASSERT_FALSE(n.showed);
}

void test_default_is_complete_false(void)
{
    notification_def n;
    TEST_ASSERT_FALSE(n.isComplete);
}

void test_default_received_attributes_zero(void)
{
    notification_def n;
    TEST_ASSERT_EQUAL_UINT8(0u, n.receivedAttributes);
}

void test_default_type_unknown(void)
{
    notification_def n;
    TEST_ASSERT_EQUAL_INT(APP_UNKNOWN, n.type);
}

// ─────────────────────────────────────────────────────────────────────────
// notification_def::reset()
// ─────────────────────────────────────────────────────────────────────────

void test_reset_clears_title(void)
{
    notification_def n;
    strncpy(n.title, "Incoming Call", sizeof(n.title));
    n.reset();
    TEST_ASSERT_EQUAL_UINT8('\0', n.title[0]);
}

void test_reset_clears_message(void)
{
    notification_def n;
    strncpy(n.message, "Call body", sizeof(n.message));
    n.reset();
    TEST_ASSERT_EQUAL_UINT8('\0', n.message[0]);
}

void test_reset_clears_bundle_id(void)
{
    notification_def n;
    strncpy(n.bundleId, "com.apple.mobilephone", sizeof(n.bundleId));
    n.reset();
    TEST_ASSERT_EQUAL_UINT8('\0', n.bundleId[0]);
}

void test_reset_clears_showed(void)
{
    notification_def n;
    n.showed = true;
    n.reset();
    TEST_ASSERT_FALSE(n.showed);
}

void test_reset_clears_is_complete(void)
{
    notification_def n;
    n.isComplete = true;
    n.reset();
    TEST_ASSERT_FALSE(n.isComplete);
}

void test_reset_clears_received_attributes(void)
{
    notification_def n;
    n.receivedAttributes = notification_def::ATTR_ALL;
    n.reset();
    TEST_ASSERT_EQUAL_UINT8(0u, n.receivedAttributes);
}

// reset() must NOT clear type or key — those are set before reset() is called
// in the firmware to preserve the notification identity during an update cycle.
void test_reset_preserves_key(void)
{
    notification_def n;
    n.key = 0xDEADBEEFu;
    n.reset();
    // key is preserved — reset() only clears string/flag fields
    TEST_ASSERT_EQUAL_UINT32(0xDEADBEEFu, n.key);
}

// ─────────────────────────────────────────────────────────────────────────
// notification_def::isCall()
// ─────────────────────────────────────────────────────────────────────────

void test_is_call_phone(void)
{
    notification_def n;
    n.type = APP_PHONE;
    TEST_ASSERT_TRUE(n.isCall());
}

void test_is_call_facetime(void)
{
    notification_def n;
    n.type = APP_FACETIME;
    TEST_ASSERT_TRUE(n.isCall());
}

void test_is_call_sms_false(void)
{
    notification_def n;
    n.type = APP_SMS;
    TEST_ASSERT_FALSE(n.isCall());
}

void test_is_call_messenger_false(void)
{
    notification_def n;
    n.type = APP_MESSENGER;
    TEST_ASSERT_FALSE(n.isCall());
}

void test_is_call_unknown_false(void)
{
    notification_def n;
    n.type = APP_UNKNOWN;
    TEST_ASSERT_FALSE(n.isCall());
}

// ─────────────────────────────────────────────────────────────────────────
// ATTR_* bitmask constants
// ─────────────────────────────────────────────────────────────────────────

void test_attr_all_is_union_of_parts(void)
{
    TEST_ASSERT_EQUAL_UINT8(
        notification_def::ATTR_TITLE | notification_def::ATTR_MESSAGE | notification_def::ATTR_DATE,
        notification_def::ATTR_ALL);
}

void test_attr_bits_are_distinct(void)
{
    TEST_ASSERT_NOT_EQUAL(notification_def::ATTR_TITLE,   notification_def::ATTR_MESSAGE);
    TEST_ASSERT_NOT_EQUAL(notification_def::ATTR_TITLE,   notification_def::ATTR_DATE);
    TEST_ASSERT_NOT_EQUAL(notification_def::ATTR_MESSAGE, notification_def::ATTR_DATE);
}

void test_attr_received_tracking(void)
{
    notification_def n;
    n.receivedAttributes = 0;

    n.receivedAttributes |= notification_def::ATTR_TITLE;
    TEST_ASSERT_FALSE(n.receivedAttributes == notification_def::ATTR_ALL);

    n.receivedAttributes |= notification_def::ATTR_MESSAGE;
    TEST_ASSERT_FALSE(n.receivedAttributes == notification_def::ATTR_ALL);

    n.receivedAttributes |= notification_def::ATTR_DATE;
    TEST_ASSERT_EQUAL_UINT8(notification_def::ATTR_ALL, n.receivedAttributes);
}

// ─────────────────────────────────────────────────────────────────────────
// Buffer size guards
// ─────────────────────────────────────────────────────────────────────────

void test_title_buffer_size(void)
{
    // title[64] — fill to capacity, verify null termination survives
    notification_def n;
    memset(n.title, 'A', sizeof(n.title) - 1);
    n.title[sizeof(n.title) - 1] = '\0';
    TEST_ASSERT_EQUAL_size_t(63u, strlen(n.title));
}

void test_message_buffer_size(void)
{
    notification_def n;
    memset(n.message, 'B', sizeof(n.message) - 1);
    n.message[sizeof(n.message) - 1] = '\0';
    TEST_ASSERT_EQUAL_size_t(127u, strlen(n.message));
}

void test_bundle_id_buffer_size(void)
{
    notification_def n;
    memset(n.bundleId, 'C', sizeof(n.bundleId) - 1);
    n.bundleId[sizeof(n.bundleId) - 1] = '\0';
    TEST_ASSERT_EQUAL_size_t(63u, strlen(n.bundleId));
}

// ─────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────
int main(void)
{
    UNITY_BEGIN();

    // Default values
    RUN_TEST(test_default_title_empty);
    RUN_TEST(test_default_message_empty);
    RUN_TEST(test_default_bundle_id_empty);
    RUN_TEST(test_default_showed_false);
    RUN_TEST(test_default_is_complete_false);
    RUN_TEST(test_default_received_attributes_zero);
    RUN_TEST(test_default_type_unknown);

    // reset()
    RUN_TEST(test_reset_clears_title);
    RUN_TEST(test_reset_clears_message);
    RUN_TEST(test_reset_clears_bundle_id);
    RUN_TEST(test_reset_clears_showed);
    RUN_TEST(test_reset_clears_is_complete);
    RUN_TEST(test_reset_clears_received_attributes);
    RUN_TEST(test_reset_preserves_key);

    // isCall()
    RUN_TEST(test_is_call_phone);
    RUN_TEST(test_is_call_facetime);
    RUN_TEST(test_is_call_sms_false);
    RUN_TEST(test_is_call_messenger_false);
    RUN_TEST(test_is_call_unknown_false);

    // ATTR_* bitmasks
    RUN_TEST(test_attr_all_is_union_of_parts);
    RUN_TEST(test_attr_bits_are_distinct);
    RUN_TEST(test_attr_received_tracking);

    // Buffer size guards
    RUN_TEST(test_title_buffer_size);
    RUN_TEST(test_message_buffer_size);
    RUN_TEST(test_bundle_id_buffer_size);

    return UNITY_END();
}
