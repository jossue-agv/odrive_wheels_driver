#include <atomic>
#include <chrono>
#include <future>

#include "gtest/gtest.h"

#include "odrive_wheels_driver/detail/latest_event_mailbox.hpp"

using namespace std::chrono_literals;
using odrive_wheels_driver::detail::LatestEventMailbox;

TEST(LatestEventMailbox, OverwritesAStaleValueInTheSameSlot) {
  LatestEventMailbox<int, 2U> mailbox;
  std::atomic<bool> stop{false};

  EXPECT_TRUE(mailbox.store(0U, 10));
  EXPECT_TRUE(mailbox.store(0U, 20));
  EXPECT_FALSE(mailbox.store(2U, 30));

  int value = 0;
  EXPECT_TRUE(mailbox.wait_take(value, stop));
  EXPECT_EQ(value, 20);
  EXPECT_TRUE(mailbox.empty());
}

TEST(LatestEventMailbox, TakesPendingSlotsInStoreOrder) {
  LatestEventMailbox<int, 2U> mailbox;
  std::atomic<bool> stop{false};

  ASSERT_TRUE(mailbox.store(1U, 20));
  ASSERT_TRUE(mailbox.store(0U, 10));

  int value = 0;
  ASSERT_TRUE(mailbox.wait_take(value, stop));
  EXPECT_EQ(value, 20);
  ASSERT_TRUE(mailbox.wait_take(value, stop));
  EXPECT_EQ(value, 10);
}

TEST(LatestEventMailbox, NotificationBeforeWaitIsNotLost) {
  LatestEventMailbox<int, 1U> mailbox;
  std::atomic<bool> stop{false};
  ASSERT_TRUE(mailbox.store(0U, 42));

  auto result = std::async(std::launch::async, [&mailbox, &stop]() {
    int value = 0;
    const bool taken = mailbox.wait_take(value, stop);
    return std::make_pair(taken, value);
  });

  ASSERT_EQ(result.wait_for(100ms), std::future_status::ready);
  const auto [taken, value] = result.get();
  EXPECT_TRUE(taken);
  EXPECT_EQ(value, 42);
}

TEST(LatestEventMailbox, StopUnblocksEmptyConsumer) {
  LatestEventMailbox<int, 1U> mailbox;
  std::atomic<bool> stop{false};

  auto result = std::async(std::launch::async, [&mailbox, &stop]() {
    int value = 0;
    return mailbox.wait_take(value, stop);
  });

  EXPECT_EQ(result.wait_for(20ms), std::future_status::timeout);
  stop.store(true);
  mailbox.notify_all();
  ASSERT_EQ(result.wait_for(100ms), std::future_status::ready);
  EXPECT_FALSE(result.get());
}
