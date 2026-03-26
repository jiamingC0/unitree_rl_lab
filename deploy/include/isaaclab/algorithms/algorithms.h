// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "onnxruntime_cxx_api.h"
#include <fmt/ranges.h>
#include <iostream>
#include <mutex>
#include <spdlog/spdlog.h>

namespace isaaclab
{

class Algorithms
{
public:
    virtual std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs) = 0;

    std::vector<float> get_action()
    {
        std::lock_guard<std::mutex> lock(act_mtx_);
        return action;
    }
    
    std::vector<float> action;
protected:
    std::mutex act_mtx_;
};

class OrtRunner : public Algorithms
{
public:
    OrtRunner(std::string model_path)
    {
        // Init Model
        env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "onnx_model");
        session_options.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);

        session = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);

        for (size_t i = 0; i < session->GetInputCount(); ++i) {
            Ort::TypeInfo input_type = session->GetInputTypeInfo(i);
            input_shapes.push_back(input_type.GetTensorTypeAndShapeInfo().GetShape());
            auto input_name = session->GetInputNameAllocated(i, allocator);
            input_names.push_back(input_name.release());
        }

        input_sizes.resize(input_shapes.size(), 0);

        // Get output shape
        Ort::TypeInfo output_type = session->GetOutputTypeInfo(0);
        output_shape = output_type.GetTensorTypeAndShapeInfo().GetShape();
        auto output_name = session->GetOutputNameAllocated(0, allocator);
        output_names.push_back(output_name.release());

        int64_t action_dim = 1;
        for (auto dim : output_shape) {
            if (dim > 0) {
                action_dim = dim;
            }
        }
        action.resize(action_dim);
    }

    std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs)
    {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

        // make sure all input names are in obs
        for (const auto& name : input_names) {
            if (obs.find(name) == obs.end()) {
                throw std::runtime_error("Input name " + std::string(name) + " not found in observations.");
            }
        }

        // Create input tensors
        std::vector<Ort::Value> input_tensors;
        for(int i(0); i<input_names.size(); ++i)
        {
            const std::string name_str(input_names[i]);
            auto& input_data = obs.at(name_str);
            auto runtime_shape = resolve_runtime_shape(input_shapes[i], input_data.size(), name_str);
            auto input_tensor = Ort::Value::CreateTensor<float>(
                memory_info,
                input_data.data(),
                input_data.size(),
                runtime_shape.data(),
                runtime_shape.size()
            );
            input_tensors.push_back(std::move(input_tensor));
        }

        // Run the model
        auto output_tensor = session->Run(Ort::RunOptions{nullptr}, input_names.data(), input_tensors.data(), input_tensors.size(), output_names.data(), 1);

        // Copy output data
        auto floatarr = output_tensor.front().GetTensorMutableData<float>();
        auto output_info = output_tensor.front().GetTensorTypeAndShapeInfo();
        const size_t output_size = output_info.GetElementCount();
        std::lock_guard<std::mutex> lock(act_mtx_);
        action.resize(output_size);
        std::memcpy(action.data(), floatarr, output_size * sizeof(float));
        return action;
    }

private:
    std::vector<int64_t> resolve_runtime_shape(const std::vector<int64_t>& declared_shape,
                                               size_t element_count,
                                               const std::string& input_name) const
    {
        std::vector<int64_t> runtime_shape = declared_shape;
        int dynamic_index = -1;
        int64_t known_product = 1;

        for (int i = 0; i < static_cast<int>(runtime_shape.size()); ++i) {
            auto& dim = runtime_shape[i];
            if (dim > 0) {
                known_product *= dim;
                continue;
            }

            if (dynamic_index != -1) {
                throw std::runtime_error(
                    "Input " + input_name + " has more than one dynamic dimension; current deploy runtime only supports one."
                );
            }
            dynamic_index = i;
            dim = 1; // default batch-like dynamic dimension
        }

        if (dynamic_index == -1) {
            int64_t shape_product = 1;
            for (auto dim : runtime_shape) {
                shape_product *= dim;
            }
            if (shape_product != static_cast<int64_t>(element_count)) {
                throw std::runtime_error(
                    "Input " + input_name + " expected " + std::to_string(shape_product)
                    + " elements, but got " + std::to_string(element_count)
                );
            }
            return runtime_shape;
        }

        if (known_product <= 0 || element_count % known_product != 0) {
            throw std::runtime_error(
                "Input " + input_name + " cannot infer dynamic dimension from " + std::to_string(element_count)
                + " elements."
            );
        }

        runtime_shape[dynamic_index] = static_cast<int64_t>(element_count / known_product);
        spdlog::info("OrtRunner: resolved input '{}' shape to [{}]", input_name, fmt::join(runtime_shape, ", "));
        return runtime_shape;
    }

    Ort::Env env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;
    Ort::AllocatorWithDefaultOptions allocator;

    std::vector<const char*> input_names;
    std::vector<const char*> output_names;

    std::vector<std::vector<int64_t>> input_shapes;
    std::vector<int64_t> input_sizes;
    std::vector<int64_t> output_shape;
};
};
