/**
 * @file runtime_assert.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Original work Copyright 2018 Oleg Y. Sinyavskiy
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Original authors: Oleg Y. Sinyavskiy
 * 
 */
 
#ifndef RUNTIME_ASSERT_HPP
#define RUNTIME_ASSERT_HPP

#define runtime_assert(EX) (void)((EX) || (throw std::runtime_error(#EX),0))

#endif
