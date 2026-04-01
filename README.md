
# Multithreaded Proxy Web Server in C++

[![C++](https://img.shields.io/badge/C%2B%2B-17-blue.svg?logo=c%2B%2B)](https://isocpp.org/)
[![Windows](https://img.shields.io/badge/Platform-Windows-lightgrey?logo=windows)](https://www.microsoft.com/windows)
[![Multithreading](https://img.shields.io/badge/Feature-Multithreaded-green)]()
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A high-performance, multithreaded HTTP proxy server built from scratch in **C++**, designed using **system design principles** and **modern C++ best practices**. This project features efficient request handling using **a custom thread pool**, modular design, **LFU/LRU caching**, **IP/domain blacklisting**, **basic authentication**, and **thread-safe logging**.[Click here for Demo Video](https://www.youtube.com/watch?v=OSm4GZx7ZQQ)

> ✅ **Strategy Pattern** used for pluggable caching strategies (LFU/LRU)  
> ✅ **Singleton Pattern** for thread-safe logging

---

## Features

| Feature                   | Description                                                                 |
|---------------------------|-----------------------------------------------------------------------------|
| 🔁 Multithreading          | Thread pool handles multiple clients concurrently                           |
| 🧠 Caching                 | LFU/LRU support via Strategy Design Pattern                                  |
| 🚫 Blacklisting           | Blocks specific IPs or domains                                              |
| 🔐 Authentication         | Basic HTTP auth using Base64 headers                                        |
| 🧾 Thread-safe Logging     | Centralized logger using Singleton pattern                                  |
| 🧱 Clean OOP Architecture  | Modular and extensible for new features                                     |
| 💻 Windows Networking API | Built on Winsock2                                                           |

---

## Architecture Overview

```text
     Client(s)
        │
        ▼
 ┌─────────────────────┐
 │    Proxy Server     │
 │ ─────────────────── │
 │ ▸ Authentication     │
 │ ▸ Blacklist Check    │
 │ ▸ LFU/LRU Cache      │
 │ ▸ Thread Pool        │
 │ ▸ Logger             │
 │ ▸ Request Forwarding │
 └─────────────────────┘
        │
        ▼
  Remote Web Server
```

---

## Folder Structure

```bash
MultiThreaded_Proxy_Web_Server/
├── include/
│   ├── Logger/
│   │   └── Logger.hpp
│   ├── ProxyServer/
│   │   └── ProxyServer.hpp
│   └── ThreadPool/
│       └── ThreadPool.hpp
│
├── src/
│   ├── main.cpp
│   ├── Client.cpp
│   ├── Logger/
│   │   └── Logger.cpp
│   ├── ProxyServer/
│   │   └── ProxyServer.cpp
│   └── ThreadPool/
│       └── ThreadPool.cpp  
│
└── README.md
```

---

## Build Instructions

### Requirements

- OS: **Windows**
- Compiler: **g++ (MinGW or Code::Blocks)**
- Dependency: `-lws2_32` (Winsock2 library)

### Build Commands

```bash
# Compile the server
g++ src/main.cpp src/ProxyServer/ProxyServer.cpp src/Logger/Logger.cpp src/ThreadPool/ThreadPool.cpp -o proxy_server -lws2_32

# Run the server
./proxy_server

# Compile the client
g++ src/Client.cpp -o client -lws2_32
./client
```

---

## 🔐 Authentication

Use HTTP **Basic Authentication** header in requests:

```http
Authorization: Basic YWRtaW46cGFzc3dvcmQ=
```

> Default:  
> - Username: `admin`  
> - Password: `password`

---

## 📚 Sample Log Output

```
[2025-06-20 11:12:05] [INFO] : Proxy server started on port 8080
[2025-06-20 11:12:07] [INFO] : Request from 127.0.0.1 → www.example.com
[2025-06-20 11:12:08] [WARNING] : Blacklisted access attempt to example.com. Blocked.
```

---

## 🧠 Design Patterns Used

- **Strategy Pattern** – Enables dynamic switching between **LFU** and **LRU** cache strategies.
- **Singleton Pattern** – Ensures a **single instance of Logger** with thread-safe access from multiple threads.

---

> ⭐ _If you found this project helpful or interesting, please give it a star and share it!_


