#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../src/planner.h"

// Test initialization of planner block
void test_planner_block_init() {
    printf("Testing planner block initialization...\n");
    
    planner_block_t block;
    
    // Initialize the block
    planner_block_init(&block);
    
    // Verify all fields are initialized correctly
    assert(block.entry_speed == 0.0f);
    assert(block.nominal_speed == 0.0f);
    assert(block.exit_speed == 0.0f);
    assert(block.acceleration == 0.0f);
    assert(block.max_entry_speed == 0.0f);
    assert(block.millimeters == 0.0f);
    assert(block.direction_bits == 0);
    assert(block.step_event_count == 0);
    assert(block.recalculate_flag == 0);
    assert(block.nominal_length_flag == 0);
    assert(block.next == NULL);
    
    printf("[passed]\n");
}

// Test validation of NULL pointer
void test_planner_block_validate_null() {
    printf("Testing planner block validation with NULL...\n");
    
    // NULL pointer should fail validation
    assert(planner_block_validate(NULL) == 0);
    
    printf("[passed]\n");
}

// Test validation of valid planner block
void test_planner_block_validate_valid() {
    printf("Testing planner block validation with valid block...\n");
    
    planner_block_t block;
    planner_block_init(&block);
    
    // Set valid values
    block.entry_speed = 100.0f;
    block.nominal_speed = 200.0f;
    block.exit_speed = 50.0f;
    block.acceleration = 500.0f;
    block.max_entry_speed = 150.0f;
    block.millimeters = 10.0f;
    block.step_event_count = 1000;
    
    // Should pass validation
    assert(planner_block_validate(&block) == 1);
    
    printf("[passed]\n");
}

// Test validation with negative entry speed
void test_planner_block_validate_negative_entry_speed() {
    printf("Testing planner block validation with negative entry speed...\n");
    
    planner_block_t block;
    planner_block_init(&block);
    
    block.entry_speed = -10.0f;  // Invalid
    block.nominal_speed = 200.0f;
    block.exit_speed = 50.0f;
    
    // Should fail validation
    assert(planner_block_validate(&block) == 0);
    
    printf("[passed]\n");
}

// Test validation with negative nominal speed
void test_planner_block_validate_negative_nominal_speed() {
    printf("Testing planner block validation with negative nominal speed...\n");
    
    planner_block_t block;
    planner_block_init(&block);
    
    block.entry_speed = 100.0f;
    block.nominal_speed = -200.0f;  // Invalid
    block.exit_speed = 50.0f;
    
    // Should fail validation
    assert(planner_block_validate(&block) == 0);
    
    printf("[passed]\n");
}

// Test validation with negative exit speed
void test_planner_block_validate_negative_exit_speed() {
    printf("Testing planner block validation with negative exit speed...\n");
    
    planner_block_t block;
    planner_block_init(&block);
    
    block.entry_speed = 100.0f;
    block.nominal_speed = 200.0f;
    block.exit_speed = -50.0f;  // Invalid
    
    // Should fail validation
    assert(planner_block_validate(&block) == 0);
    
    printf("[passed]\n");
}

// Test validation with negative acceleration
void test_planner_block_validate_negative_acceleration() {
    printf("Testing planner block validation with negative acceleration...\n");
    
    planner_block_t block;
    planner_block_init(&block);
    
    block.entry_speed = 100.0f;
    block.nominal_speed = 200.0f;
    block.exit_speed = 50.0f;
    block.acceleration = -500.0f;  // Invalid
    
    // Should fail validation
    assert(planner_block_validate(&block) == 0);
    
    printf("[passed]\n");
}

// Test validation with negative distance
void test_planner_block_validate_negative_distance() {
    printf("Testing planner block validation with negative distance...\n");
    
    planner_block_t block;
    planner_block_init(&block);
    
    block.entry_speed = 100.0f;
    block.nominal_speed = 200.0f;
    block.exit_speed = 50.0f;
    block.millimeters = -10.0f;  // Invalid
    
    // Should fail validation
    assert(planner_block_validate(&block) == 0);
    
    printf("[passed]\n");
}

// Test validation with entry speed exceeding max entry speed
void test_planner_block_validate_entry_exceeds_max() {
    printf("Testing planner block validation with entry speed exceeding max...\n");
    
    planner_block_t block;
    planner_block_init(&block);
    
    block.entry_speed = 200.0f;  // Exceeds max
    block.max_entry_speed = 150.0f;
    block.nominal_speed = 300.0f;
    block.exit_speed = 50.0f;
    
    // Should fail validation
    assert(planner_block_validate(&block) == 0);
    
    printf("[passed]\n");
}

// Test validation with entry speed exceeding nominal speed
void test_planner_block_validate_entry_exceeds_nominal() {
    printf("Testing planner block validation with entry speed exceeding nominal...\n");
    
    planner_block_t block;
    planner_block_init(&block);
    
    block.entry_speed = 250.0f;  // Exceeds nominal
    block.nominal_speed = 200.0f;
    block.exit_speed = 50.0f;
    
    // Should fail validation
    assert(planner_block_validate(&block) == 0);
    
    printf("[passed]\n");
}

// Test validation with exit speed exceeding nominal speed
void test_planner_block_validate_exit_exceeds_nominal() {
    printf("Testing planner block validation with exit speed exceeding nominal...\n");
    
    planner_block_t block;
    planner_block_init(&block);
    
    block.entry_speed = 100.0f;
    block.nominal_speed = 200.0f;
    block.exit_speed = 250.0f;  // Exceeds nominal
    
    // Should fail validation
    assert(planner_block_validate(&block) == 0);
    
    printf("[passed]\n");
}

// Test that all required fields exist in the structure
void test_planner_block_has_required_fields() {
    printf("Testing that planner block has all required fields...\n");
    
    planner_block_t block;
    
    // Test that we can access all required fields
    block.entry_speed = 100.0f;
    block.nominal_speed = 200.0f;
    block.exit_speed = 50.0f;
    block.acceleration = 500.0f;
    block.max_entry_speed = 150.0f;
    block.millimeters = 10.0f;
    block.direction_bits = 0xFF;
    block.step_event_count = 1000;
    block.recalculate_flag = 1;
    block.nominal_length_flag = 1;
    block.next = NULL;
    
    // Verify values were set correctly
    assert(block.entry_speed == 100.0f);
    assert(block.nominal_speed == 200.0f);
    assert(block.exit_speed == 50.0f);
    assert(block.acceleration == 500.0f);
    assert(block.max_entry_speed == 150.0f);
    assert(block.millimeters == 10.0f);
    assert(block.direction_bits == 0xFF);
    assert(block.step_event_count == 1000);
    assert(block.recalculate_flag == 1);
    assert(block.nominal_length_flag == 1);
    assert(block.next == NULL);
    
    printf("[passed]\n");
}

// Test planner block with zero nominal speed (edge case)
void test_planner_block_zero_nominal_speed() {
    printf("Testing planner block with zero nominal speed...\n");
    
    planner_block_t block;
    planner_block_init(&block);
    
    // Zero nominal speed should be valid
    // (validation only checks if entry/exit exceed nominal when nominal > 0)
    block.entry_speed = 0.0f;
    block.nominal_speed = 0.0f;
    block.exit_speed = 0.0f;
    block.acceleration = 100.0f;
    
    assert(planner_block_validate(&block) == 1);
    
    printf("[passed]\n");
}

// Test planner block with complete stop (all speeds zero)
void test_planner_block_complete_stop() {
    printf("Testing planner block with complete stop (all speeds zero)...\n");
    
    planner_block_t block;
    planner_block_init(&block);
    
    // All speeds zero should be valid (represents a complete stop)
    block.entry_speed = 0.0f;
    block.nominal_speed = 0.0f;
    block.exit_speed = 0.0f;
    
    assert(planner_block_validate(&block) == 1);
    
    printf("[passed]\n");
}

// Main function to execute all test cases
int main() {
    printf("=== Running Planner Block Tests ===\n\n");
    
    // Run all tests
    test_planner_block_init();
    test_planner_block_validate_null();
    test_planner_block_validate_valid();
    test_planner_block_validate_negative_entry_speed();
    test_planner_block_validate_negative_nominal_speed();
    test_planner_block_validate_negative_exit_speed();
    test_planner_block_validate_negative_acceleration();
    test_planner_block_validate_negative_distance();
    test_planner_block_validate_entry_exceeds_max();
    test_planner_block_validate_entry_exceeds_nominal();
    test_planner_block_validate_exit_exceeds_nominal();
    test_planner_block_has_required_fields();
    test_planner_block_zero_nominal_speed();
    test_planner_block_complete_stop();
    
    printf("\n=== All planner block tests passed! ===\n");
    return 0;
}
