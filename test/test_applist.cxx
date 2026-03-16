/**
 * test_applist.cxx — Unity tests for ApplicationList logic.
 *
 * applist.cxx uses FreeRTOS semaphores and NVS persistence.  Both are
 * replaced by the minimal stubs in test/stubs/ so the pure logic runs
 * on the host.  NVS stubs return ESP_ERR_NVS_NOT_FOUND, so _loadFromNvs
 * exits immediately and the list starts empty (only built-ins in flash).
 */

#include "unity.h"
#include "applist.h"   // pulls in freertos/semphr.h and nvs_flash.h via stubs/

#include <cstring>

void setUp(void)    {}
void tearDown(void) {}

// A fresh ApplicationList is constructed per test to avoid cross-test state.
// The constructor calls _loadFromNvs which exits early (NVS stub returns NOT_FOUND).

// ─────────────────────────────────────────────────────────────────────────
// Built-in lookup tests
// ─────────────────────────────────────────────────────────────────────────

void test_builtin_sms_is_allowed(void)
{
    ApplicationList al;
    TEST_ASSERT_TRUE(al.isAllowedApplication("com.apple.MobileSMS"));
}

void test_builtin_phone_is_allowed(void)
{
    ApplicationList al;
    TEST_ASSERT_TRUE(al.isAllowedApplication("com.apple.mobilephone"));
}

void test_builtin_signal_is_allowed(void)
{
    ApplicationList al;
    TEST_ASSERT_TRUE(al.isAllowedApplication("org.whispersystems.signal"));
}

void test_unknown_bundle_id_not_allowed(void)
{
    ApplicationList al;
    TEST_ASSERT_FALSE(al.isAllowedApplication("com.example.unknownapp"));
}

void test_empty_bundle_id_not_allowed(void)
{
    ApplicationList al;
    TEST_ASSERT_FALSE(al.isAllowedApplication(""));
}

void test_get_application_id_sms(void)
{
    ApplicationList al;
    TEST_ASSERT_EQUAL_INT(APP_SMS, al.getApplicationId("com.apple.MobileSMS"));
}

void test_get_application_id_phone(void)
{
    ApplicationList al;
    TEST_ASSERT_EQUAL_INT(APP_PHONE, al.getApplicationId("com.apple.mobilephone"));
}

void test_get_application_id_unknown(void)
{
    ApplicationList al;
    TEST_ASSERT_EQUAL_INT(APP_UNKNOWN, al.getApplicationId("com.example.nope"));
}

void test_get_display_name_by_enum_sms(void)
{
    ApplicationList al;
    TEST_ASSERT_EQUAL_STRING("iMessage", al.getDisplayName(APP_SMS));
}

void test_get_display_name_by_enum_phone(void)
{
    ApplicationList al;
    TEST_ASSERT_EQUAL_STRING("Call", al.getDisplayName(APP_PHONE));
}

void test_get_display_name_by_enum_facetime(void)
{
    ApplicationList al;
    TEST_ASSERT_EQUAL_STRING("Facetime", al.getDisplayName(APP_FACETIME));
}

void test_get_display_name_by_bundle_sms(void)
{
    ApplicationList al;
    TEST_ASSERT_EQUAL_STRING("iMessage", al.getDisplayName("com.apple.MobileSMS"));
}

void test_get_display_name_by_bundle_unknown_returns_bundle_id(void)
{
    ApplicationList al;
    // Unknown bundle IDs fall back to returning the bundle ID itself
    const char* result = al.getDisplayName("com.example.myapp");
    TEST_ASSERT_EQUAL_STRING("com.example.myapp", result);
}

void test_is_builtin_true_for_known(void)
{
    ApplicationList al;
    TEST_ASSERT_TRUE(al.isBuiltIn("com.apple.MobileSMS"));
}

void test_is_builtin_false_for_unknown(void)
{
    ApplicationList al;
    TEST_ASSERT_FALSE(al.isBuiltIn("com.example.unknownapp"));
}

// ─────────────────────────────────────────────────────────────────────────
// Custom entry management
// ─────────────────────────────────────────────────────────────────────────

void test_add_custom_entry_then_allowed(void)
{
    ApplicationList al;
    TEST_ASSERT_FALSE(al.isAllowedApplication("com.example.myapp"));
    TEST_ASSERT_TRUE(al.addEntry("com.example.myapp", "My App"));
    TEST_ASSERT_TRUE(al.isAllowedApplication("com.example.myapp"));
}

void test_add_custom_entry_display_name(void)
{
    ApplicationList al;
    al.addEntry("com.example.myapp", "My Custom App");
    TEST_ASSERT_EQUAL_STRING("My Custom App", al.getDisplayName("com.example.myapp"));
}

void test_add_custom_overrides_builtin_display_name(void)
{
    ApplicationList al;
    // Built-in "iMessage" — add a custom entry with the same bundle ID is rejected
    // (isBuiltIn check), so actually a custom can't override a built-in display name
    // via addEntry. Verify the rejection.
    TEST_ASSERT_FALSE(al.addEntry("com.apple.MobileSMS", "My iMessage"));
    // Original display name unchanged
    TEST_ASSERT_EQUAL_STRING("iMessage", al.getDisplayName("com.apple.MobileSMS"));
}

void test_add_duplicate_custom_returns_false(void)
{
    ApplicationList al;
    TEST_ASSERT_TRUE(al.addEntry("com.example.myapp", "My App"));
    TEST_ASSERT_FALSE(al.addEntry("com.example.myapp", "My App Again"));
    // Count still 1
    TEST_ASSERT_EQUAL_size_t(1u, al.getCustomCount());
}

void test_add_builtin_bundle_id_returns_false(void)
{
    ApplicationList al;
    TEST_ASSERT_FALSE(al.addEntry("com.apple.mobilephone", "Phone Override"));
}

void test_remove_custom_entry(void)
{
    ApplicationList al;
    al.addEntry("com.example.myapp", "My App");
    TEST_ASSERT_TRUE(al.isAllowedApplication("com.example.myapp"));
    TEST_ASSERT_TRUE(al.removeEntry("com.example.myapp"));
    TEST_ASSERT_FALSE(al.isAllowedApplication("com.example.myapp"));
    TEST_ASSERT_EQUAL_size_t(0u, al.getCustomCount());
}

void test_remove_nonexistent_returns_false(void)
{
    ApplicationList al;
    TEST_ASSERT_FALSE(al.removeEntry("com.example.nobody"));
}

void test_remove_builtin_returns_false(void)
{
    ApplicationList al;
    TEST_ASSERT_FALSE(al.removeEntry("com.apple.MobileSMS"));
    // Built-in still accessible
    TEST_ASSERT_TRUE(al.isAllowedApplication("com.apple.MobileSMS"));
}

void test_custom_count_tracks_add_remove(void)
{
    ApplicationList al;
    TEST_ASSERT_EQUAL_size_t(0u, al.getCustomCount());
    al.addEntry("com.example.a", "App A");
    TEST_ASSERT_EQUAL_size_t(1u, al.getCustomCount());
    al.addEntry("com.example.b", "App B");
    TEST_ASSERT_EQUAL_size_t(2u, al.getCustomCount());
    al.removeEntry("com.example.a");
    TEST_ASSERT_EQUAL_size_t(1u, al.getCustomCount());
}

void test_get_custom_entry_by_index(void)
{
    ApplicationList al;
    al.addEntry("com.example.first",  "First");
    al.addEntry("com.example.second", "Second");

    ApplicationList::CustomEntry e = {};
    TEST_ASSERT_TRUE(al.getCustomEntry(0, e));
    TEST_ASSERT_EQUAL_STRING("com.example.first", e.bundleId);
    TEST_ASSERT_EQUAL_STRING("First", e.displayName);

    TEST_ASSERT_TRUE(al.getCustomEntry(1, e));
    TEST_ASSERT_EQUAL_STRING("com.example.second", e.bundleId);

    TEST_ASSERT_FALSE(al.getCustomEntry(2, e)); // out of bounds
}

void test_reset_to_defaults_clears_custom(void)
{
    ApplicationList al;
    al.addEntry("com.example.myapp", "My App");
    TEST_ASSERT_EQUAL_size_t(1u, al.getCustomCount());
    al.resetToDefaults();
    TEST_ASSERT_EQUAL_size_t(0u, al.getCustomCount());
    TEST_ASSERT_FALSE(al.isAllowedApplication("com.example.myapp"));
    // Built-ins are unaffected
    TEST_ASSERT_TRUE(al.isAllowedApplication("com.apple.MobileSMS"));
}

void test_list_full_rejects_new_entry(void)
{
    ApplicationList al;
    // Fill all MAX_CUSTOM_ENTRIES slots
    char bundle[64]; char name[32];
    for (size_t i = 0; i < ApplicationList::MAX_CUSTOM_ENTRIES; i++) {
        snprintf(bundle, sizeof(bundle), "com.example.app%zu", i);
        snprintf(name,   sizeof(name),   "App %zu", i);
        TEST_ASSERT_TRUE(al.addEntry(bundle, name));
    }
    TEST_ASSERT_EQUAL_size_t(ApplicationList::MAX_CUSTOM_ENTRIES, al.getCustomCount());
    // One more must be rejected
    TEST_ASSERT_FALSE(al.addEntry("com.example.overflow", "Overflow"));
}

// ─────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────
int main(void)
{
    UNITY_BEGIN();

    // Built-in lookups
    RUN_TEST(test_builtin_sms_is_allowed);
    RUN_TEST(test_builtin_phone_is_allowed);
    RUN_TEST(test_builtin_signal_is_allowed);
    RUN_TEST(test_unknown_bundle_id_not_allowed);
    RUN_TEST(test_empty_bundle_id_not_allowed);
    RUN_TEST(test_get_application_id_sms);
    RUN_TEST(test_get_application_id_phone);
    RUN_TEST(test_get_application_id_unknown);
    RUN_TEST(test_get_display_name_by_enum_sms);
    RUN_TEST(test_get_display_name_by_enum_phone);
    RUN_TEST(test_get_display_name_by_enum_facetime);
    RUN_TEST(test_get_display_name_by_bundle_sms);
    RUN_TEST(test_get_display_name_by_bundle_unknown_returns_bundle_id);
    RUN_TEST(test_is_builtin_true_for_known);
    RUN_TEST(test_is_builtin_false_for_unknown);

    // Custom entry management
    RUN_TEST(test_add_custom_entry_then_allowed);
    RUN_TEST(test_add_custom_entry_display_name);
    RUN_TEST(test_add_custom_overrides_builtin_display_name);
    RUN_TEST(test_add_duplicate_custom_returns_false);
    RUN_TEST(test_add_builtin_bundle_id_returns_false);
    RUN_TEST(test_remove_custom_entry);
    RUN_TEST(test_remove_nonexistent_returns_false);
    RUN_TEST(test_remove_builtin_returns_false);
    RUN_TEST(test_custom_count_tracks_add_remove);
    RUN_TEST(test_get_custom_entry_by_index);
    RUN_TEST(test_reset_to_defaults_clears_custom);
    RUN_TEST(test_list_full_rejects_new_entry);

    return UNITY_END();
}
