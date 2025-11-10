#pragma once

#include <string>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <iostream>
#include <optional>

class HttpClient {
public:
    HttpClient() {
        curl_global_init(CURL_GLOBAL_DEFAULT);
        curl = curl_easy_init();
        if (!curl) throw std::runtime_error("Failed to init curl");
    }

    ~HttpClient() {
        if (curl) curl_easy_cleanup(curl);
        curl_global_cleanup();
    }

    // ===== GET =====
    std::string get(const std::string &url) {
        return performRequest(url, "GET");
    }

    std::optional<nlohmann::json> getJson(const std::string &url) {
        try {
            return nlohmann::json::parse(get(url));
        } catch (const nlohmann::json::parse_error &e) {
            std::cerr << "JSON parse error: " << e.what() << std::endl;
            return std::nullopt;
        }
    }

    // ===== POST =====
    std::string post(const std::string &url, const std::string &body) {
        return performRequest(url, "POST", body);
    }

    std::string post(const std::string &url, const nlohmann::json &j) {
        return performRequest(url, "POST", j.dump(), true);
    }

    std::optional<nlohmann::json> postJson(const std::string &url, const nlohmann::json &j) {
        try {
            return nlohmann::json::parse(post(url, j));
        } catch (...) {
            return std::nullopt;
        }
    }

    // ===== PUT =====
    std::string put(const std::string &url, const std::string &body) {
        return performRequest(url, "PUT", body);
    }

    std::string put(const std::string &url, const nlohmann::json &j) {
        return performRequest(url, "PUT", j.dump(), true);
    }

    std::optional<nlohmann::json> putJson(const std::string &url, const nlohmann::json &j) {
        try {
            return nlohmann::json::parse(put(url, j));
        } catch (...) {
            return std::nullopt;
        }
    }

private:
    CURL* curl;

    static size_t writeCallback(void *contents, size_t size, size_t nmemb, void *userp) {
        size_t total = size * nmemb;
        ((std::string*)userp)->append((char*)contents, total);
        return total;
    }

    std::string performRequest(const std::string &url,
                               const std::string &method,
                               const std::string &body = "",
                               bool json = false)
    {   
        try{
            if (!curl) throw std::runtime_error("CURL not initialized");

            curl_easy_reset(curl);

            std::string response;
            struct curl_slist *headers = nullptr;

            if (json)
                headers = curl_slist_append(headers, "Content-Type: application/json");

            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);
            curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

            if (method == "POST") {
                curl_easy_setopt(curl, CURLOPT_POST, 1L);
                curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
            } else if (method == "PUT") {
                curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
                curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
            }

            if (headers)
                curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

            CURLcode res = curl_easy_perform(curl);

            if (headers) curl_slist_free_all(headers);

            if (res != CURLE_OK) throw std::runtime_error(curl_easy_strerror(res));

            return response;
        }
        catch (const std::exception &e) {
            std::cerr << "HTTP request failed: " << e.what() << std::endl;
            return "";
        } catch (...) {
            std::cerr << "HTTP request failed: unknown error" << std::endl;
            return "";
        }
        
    }
};
