/*
 * This file is part of a c++ utility library for platforms where no
 * full c++ standard library is available.
 * Copyright (c) 2018 Timo Sandmann
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    functor.h
 * @brief   Simplified general-purpose polymorphic function wrapper
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_FUNCTOR_H_
#define SRC_FUNCTOR_H_

#include <type_traits>
#include <cstdint>
#include <cstring>

#include <iostream>


template <typename T, typename... BoundArgs>
class Functor;

/**
 * @brief General-purpose polymorphic function wrapper
 * @tparam Res: Result type of wrapped function
 * @tparam Args: Variadic argument types of wrapped function
 */
template <typename Res, typename... Args>
class Functor<Res(Args...)> {
public:
    using type = Res(*)(void*, Args...);

protected:
    void* le_;
    uint8_t* p_lambda_;
    size_t size;
    type func_;

    /**
     * @brief Helper to call a function pointer with given args
     * @param[in] v: void-Pointer to a function of type Res(Args...)
     * @param args: Arguments for function to call
     * @return Result of function calls or default-constructed Res if v is nullptr
     */
    static Res func_ptr_exec(void* v, Args... args) {
        if (v) {
            auto p_func(reinterpret_cast<Res(*)(Args...)>(v));
            return (*p_func)(args...);
        } else {
            return Res();
        }
    }

    /**
     * @brief Helper to execute a lambda expression with given args
     * @tparam U: Type of lambda to call
     * @param[in] v: Pointer to a stored lambda
     * @param args: Arguments for lambda to execute
     * @return Result of lambda expression
     * @note Allocated heap memory is owned by le_
     */
    template <typename U>
    static Res lambda_ptr_exec(U* v, Args... args) {
        // std::cout << "Functor::lambda_ptr_exec(): v=0x" << std::hex << reinterpret_cast<intptr_t>(v) << std::dec << "\n";
        return (*v)(args...);
    }

    /**
     * @brief Helper to store a lambda expression accessible as plain function pointer
     * @tparam U: Type of lambda
     * @param[in] u: Reference to a lambda
     * @return Function pointer that points to a dynamically stored lambda expression
     */
    template <typename U>
    type lambda_ptr(U& u) {
        p_lambda_ = new uint8_t[sizeof(U)];
        if (! p_lambda_) {
            // std::cerr << "Functor::Functor(): allocation of " << sizeof(U) << " bytes failed\n";
            size = 0;
            le_ = nullptr;
            return reinterpret_cast<type>(lambda_ptr_exec<U>);
        }
        size = sizeof(U);
        // std::cout << "Functor::lambda_ptr(): allocated " << sizeof(U) << " bytes of memory, this=0x" << std::hex << reinterpret_cast<intptr_t>(this) << std::dec << "\n";
        std::memcpy(p_lambda_, &u, sizeof(U));
        le_ = p_lambda_;
        return reinterpret_cast<type>(lambda_ptr_exec<U>);
    }

public:
    /**
     * @brief Constructs a new Functor object
     */
    Functor() noexcept : le_(nullptr), p_lambda_(nullptr), size(0), func_(func_ptr_exec) {
        // std::cout << "Functor(): empty functor created, this=0x" << std::hex << reinterpret_cast<intptr_t>(this) << std::dec << "\n";
    }

    /**
     * @brief Copy constructor
     * @param[in] other: Instance to copy
     */
    Functor(const Functor& other) {
        if (other.size) {
            p_lambda_ = new uint8_t[other.size];
            if (! p_lambda_) {
                // std::cerr << "Functor::Functor(): allocation of " << other.size << " bytes failed\n";
                size = 0;
                le_ = nullptr;
                func_ = func_ptr_exec;
                return;
            }
            size = other.size;
            // std::cout << "Functor::Functor(): allocated " << size << " bytes of memory, this=0x" << std::hex << reinterpret_cast<intptr_t>(this) << ", other=0x" << reinterpret_cast<intptr_t>(&other) << std::dec << "\n";
            std::memcpy(p_lambda_, other.p_lambda_, size);
            le_ = p_lambda_;
            func_ = other.func_;
        } else {
            p_lambda_ = nullptr;
            size = 0;
            le_ = other.le_;
            func_ = func_ptr_exec;
        }
        // std::cout << "Functor(): functor copied, this=0x" << std::hex << reinterpret_cast<intptr_t>(this) << ", other=0x" << reinterpret_cast<intptr_t>(&other) << std::dec << "\n";
    }

    // /*
    //  * @brief Move constructor
    //  * @param[in] other: Instance to move in
    //  */
    // Functor(Functor&& other) noexcept {
    //     le_ = other.le_;
    //     other.le_ = nullptr;
    //     p_lambda_ = other.p_lambda_;
    //     other.p_lambda_ = nullptr;
    //     size = other.size;
    //     other.size = 0;
    //     func_ = other.func_;
    //     std::cout << "Functor(): functor moved, this=0x" << std::hex << reinterpret_cast<intptr_t>(this) << ", other=0x" << reinterpret_cast<intptr_t>(&other) << std::dec << "\n";
    // }

    /**
     * @brief Copy assignment operator
     * @param[in] other: Instance to copy
     * @return Reference to this instance
     */
    Functor& operator= (const Functor& other) {
        if (size) {
            delete[] p_lambda_;
            // std::cout << "Functor::operator=(): p_lambda_ deleted, size=" << size << ", this=0x" << std::hex << reinterpret_cast<intptr_t>(this) << ", other=0x" << reinterpret_cast<intptr_t>(&other) << std::dec << "\n";
        }

        if (other.size) {
            p_lambda_ = new uint8_t[other.size];
            if (! p_lambda_) {
                // std::cerr << "Functor::operator=(): allocation of " << other.size << " bytes failed\n";
                size = 0;
                le_ = nullptr;
                func_ = func_ptr_exec;
                return *this;
            }
            size = other.size;
            // std::cout << "Functor::operator=(): allocated " << size << " bytes of memory, this=0x" << std::hex << reinterpret_cast<intptr_t>(this) << ", other=0x" << reinterpret_cast<intptr_t>(&other) << std::dec << "\n";
            std::memcpy(p_lambda_, other.p_lambda_, size);
            le_ = p_lambda_;
            func_ = other.func_;
        } else {
            p_lambda_ = nullptr;
            size = 0;
            le_ = other.le_;
            func_ = func_ptr_exec;
        }

        // std::cout << "Functor::operator=(): functor assigned, this=0x" << std::hex << reinterpret_cast<intptr_t>(this) << ", other=0x" << reinterpret_cast<intptr_t>(&other) << std::dec << "\n";
        return *this;
    }

    // /*
    //  * @brief Move assignment operator
    //  * @param[in] other: Instance to move in
    //  * @return Reference to this instance
    //  */
    // Functor& operator= (Functor&& other) noexcept {
    //     if (this == &other) {
    //         // take precautions against `foo = std::move(foo)`
    //         return *this;
    //     }
    //     if (size) {
    //         delete[] p_lambda_;
    //         std::cout << "Functor::operator=(&&): p_lambda_ deleted, size=" << size << ", this=0x" << std::hex << reinterpret_cast<intptr_t>(this) << ", other=0x" << reinterpret_cast<intptr_t>(&other) << std::dec << "\n";
    //     }
    //     le_ = other.le_;
    //     other.le_ = nullptr;
    //     p_lambda_ = other.p_lambda_;
    //     other.p_lambda_ = nullptr;
    //     size = other.size;
    //     other.size = 0;
    //     func_ = other.func_;
    //     std::cout << "Functor(): functor move-assigned, this=0x" << std::hex << reinterpret_cast<intptr_t>(this) << ", other=0x" << reinterpret_cast<intptr_t>(&other) << std::dec << "\n";
    //     return *this;
    // }

    /**
     * @brief Destroys the Functor object
     */
    ~Functor() noexcept {
        if (p_lambda_) {
            delete[] p_lambda_;
            // std::cout << "~Functor(): p_lambda_ deleted, size=" << size << ", this=0x" << std::hex << reinterpret_cast<intptr_t>(this) << std::dec << "\n";
        }
    }

    /**
     * @brief Constructs a new Functor object from a function pointer
     * @param[in] f: The function pointer to use
     */
    Functor(Res(*f)(Args...)) noexcept : le_(reinterpret_cast<void*>(f)), p_lambda_(nullptr), size(0), func_(func_ptr_exec) {
        // std::cout << "Functor(): functor created with function pointer, size=" << size << ", this=0x" << std::hex << reinterpret_cast<intptr_t>(this) << std::dec << "\n";
    }

    /**
     * @brief Constructs a new Functor object from a lambda expression
     * @tparam U: Type of lambda
     * @param[in] v: Lambda expression to use
     */
    template <typename U>
    Functor(U&& v) noexcept : func_(lambda_ptr(v)) {
        // std::cout << "Functor(): functor created with lambda, size=" << size << ", this=0x" << std::hex << reinterpret_cast<intptr_t>(this) << std::dec << "\n";
    }

    /**
     * @brief Executes this functor by calling the referenced function or lambda
     * @param args: Arguments to use
     * @return Result of functor execution
     */
    Res operator()(Args... args) const {
        // std::cout << "Functor::operator()(): executing func_(), size=" << size << ", this=0x" << std::hex << reinterpret_cast<intptr_t>(this) << std::dec << "\n";
        // std::cout << "Functor::operator()(): func_=0x" << std::hex << reinterpret_cast<intptr_t>(func_) << ", le_=0x" << reinterpret_cast<intptr_t>(le_) << std::dec << "\n";
        return func_(le_, args...);
    }
};


/**
 * @brief General-purpose polymorphic function wrapper with bound arguments
 * @tparam Res: Result type of wrapped function
 * @tparam Args: Variadic argument types of wrapped function
 * @tparam BoundArgs: Variadic argument types of bound arguments
 */
template <typename Res, typename... Args, typename... BoundArgs>
class Functor<Res(Args...), BoundArgs...> {
protected:
    Functor<Res(Args...)> func_;

public:
    /**
     * @brief Constructs a new Functor object
     */
    Functor() noexcept {
        // std::cout << "Functor(): empty functor with bound args created\n";
    }

    /**
     * @brief Constructs a new Functor object from a function pointer
     * @param[in] f: The function pointer to use
     * @param to_bind: Parameters to bind
     */
    Functor(Res(*f)(Args..., BoundArgs&&...), BoundArgs&&... to_bind) noexcept : func_(
        [f, to_bind...](Args... args) {
            // std::cout << "Functor(): executing functor with bound args, f=" << std::hex << reinterpret_cast<void*>(f) << std::dec << "\n";
            return f(args..., to_bind...); }
    ) {
        // std::cout << "Functor(): functor with bound args created with function pointer\n";
    }

    /**
     * @brief Constructs a new Functor object from a lambda expression
     * @tparam U: Type of lambda
     * @param[in] v: Lambda expression to use
     * @param to_bind: Parameters to bind
     */
    template <typename U>
    Functor(U&& v, BoundArgs&&... to_bind) noexcept : func_([&v, to_bind...](Args... args) { return v(args..., std::forward<BoundArgs&&...>(to_bind...)); }) {
        // std::cout << "Functor(): functor with bound args created with lambda\n";
    }

    /**
     * @brief Executes this functor by calling the referenced function or lambda
     * @param args: Arguments to use
     * @return Result of functor execution
     */
    Res operator()(Args... args) const {
        // std::cout << "Functor(): executing func_()...\n";
        return func_(args...);
    }
};

#endif /* SRC_FUNCTOR_H_ */
