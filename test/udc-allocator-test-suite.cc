/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

// Include a header file from your module to test.
#include "ns3/udc-allocator.h"

// An essential include is test.h
#include "ns3/test.h"

// Do not put your test classes in namespace ns3.  You may find it useful
// to use the using directive to access the ns3 namespace directly
using namespace ns3;

// This is an example TestCase.
class UdcAllocatorTestCase1 : public TestCase
{
public:
  UdcAllocatorTestCase1 ();
  virtual ~UdcAllocatorTestCase1 ();

private:
  virtual void DoRun (void);
};

// Add some help text to this case to describe what it is intended to test
UdcAllocatorTestCase1::UdcAllocatorTestCase1 ()
  : TestCase ("UdcAllocator test case (does nothing)")
{
}

// This destructor does nothing but we include it as a reminder that
// the test case should clean up after itself
UdcAllocatorTestCase1::~UdcAllocatorTestCase1 ()
{
}

//
// This method is the pure virtual method from class TestCase that every
// TestCase must implement
//
void
UdcAllocatorTestCase1::DoRun (void)
{
  // A wide variety of test macros are available in src/core/test.h
  NS_TEST_ASSERT_MSG_EQ (true, true, "true doesn't equal true for some reason");
  // Use this one for floating point comparisons
  NS_TEST_ASSERT_MSG_EQ_TOL (0.01, 0.01, 0.001, "Numbers are not equal within tolerance");
}

// The TestSuite class names the TestSuite, identifies what type of TestSuite,
// and enables the TestCases to be run.  Typically, only the constructor for
// this class must be defined
//
class UdcAllocatorTestSuite : public TestSuite
{
public:
  UdcAllocatorTestSuite ();
};

UdcAllocatorTestSuite::UdcAllocatorTestSuite ()
  : TestSuite ("udc-allocator", UNIT)
{
  // TestDuration for TestCase can be QUICK, EXTENSIVE or TAKES_FOREVER
  AddTestCase (new UdcAllocatorTestCase1, TestCase::QUICK);
}

// Do not forget to allocate an instance of this TestSuite
static UdcAllocatorTestSuite udcAllocatorTestSuite;

