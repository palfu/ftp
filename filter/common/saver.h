#pragma once

#include "json/json.h"
#include <fstream>
#include <string>

namespace ftp
{

static bool saveJson(std::string const &filename, Json::Value const &value)
{
    Json::StreamWriterBuilder writerBuilder;  // 新式API
    std::ofstream os(filename, std::ios::out);
    if (!os.is_open()) {
        SLOG_ERROR("cannot open file %s", filename.c_str());
        return false;
    }

    std::unique_ptr<Json::StreamWriter> jsonWriter(writerBuilder.newStreamWriter());
    jsonWriter->write(value, &os);  // json-->stringstream
    os.close();
    return true;
}

class Saver
{
public:
    virtual Json::Value toJson() const
    {
        Json::Value root;
        return root;
    }

    bool saveJson(std::string filename)
    {
        Json::Value root = toJson();
        return saveJson(filename, root);
    }

    static bool saveJson(std::string filename, Json::Value const &value)
    {
        Json::StreamWriterBuilder writerBuilder;  // 新式API
        std::ofstream os(filename, std::ios::out);
        if (!os.is_open()) {
            SLOG_ERROR("cannot open file %s", filename.c_str());
            return false;
        }

        std::unique_ptr<Json::StreamWriter> jsonWriter(writerBuilder.newStreamWriter());
        jsonWriter->write(value, &os);  // json-->stringstream
        os.close();
        return true;
    }
};

}  // namespace ftp