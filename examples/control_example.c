/**
 * @file control_example.c
 * @brief Breeze框架控制模块的使用示例
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../include/breeze/breeze.h"

/* 模糊控制器示例函数 */
void fuzzy_controller_example(void) {
    BreezeFuzzyController controller;
    BreezeFuzzyMembership error_memberships[3];
    BreezeFuzzyMembership error_change_memberships[3];
    BreezeFuzzyMembership output_memberships[3];
    BreezeFuzzyRule rules[9];
    float error, error_change, output;
    int i;
    
    printf("\n模糊控制器示例\n");
    printf("------------------------------------\n");
    
    /* 初始化模糊控制器 */
    BreezeFuzzyController_Init(&controller, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, 100);
    
    /* 设置误差的模糊集合（负、零、正） */
    error_memberships[0].type = BREEZE_FUZZY_TRIANGULAR;
    error_memberships[0].params[0] = -1.5f;
    error_memberships[0].params[1] = -1.0f;
    error_memberships[0].params[2] = 0.0f;
    error_memberships[0].name = "负";
    
    error_memberships[1].type = BREEZE_FUZZY_TRIANGULAR;
    error_memberships[1].params[0] = -0.5f;
    error_memberships[1].params[1] = 0.0f;
    error_memberships[1].params[2] = 0.5f;
    error_memberships[1].name = "零";
    
    error_memberships[2].type = BREEZE_FUZZY_TRIANGULAR;
    error_memberships[2].params[0] = 0.0f;
    error_memberships[2].params[1] = 1.0f;
    error_memberships[2].params[2] = 1.5f;
    error_memberships[2].name = "正";
    
    /* 设置误差变化率的模糊集合（负、零、正） */
    error_change_memberships[0].type = BREEZE_FUZZY_TRIANGULAR;
    error_change_memberships[0].params[0] = -1.5f;
    error_change_memberships[0].params[1] = -1.0f;
    error_change_memberships[0].params[2] = 0.0f;
    error_change_memberships[0].name = "负";
    
    error_change_memberships[1].type = BREEZE_FUZZY_TRIANGULAR;
    error_change_memberships[1].params[0] = -0.5f;
    error_change_memberships[1].params[1] = 0.0f;
    error_change_memberships[1].params[2] = 0.5f;
    error_change_memberships[1].name = "零";
    
    error_change_memberships[2].type = BREEZE_FUZZY_TRIANGULAR;
    error_change_memberships[2].params[0] = 0.0f;
    error_change_memberships[2].params[1] = 1.0f;
    error_change_memberships[2].params[2] = 1.5f;
    error_change_memberships[2].name = "正";
    
    /* 设置输出的模糊集合（负、零、正） */
    output_memberships[0].type = BREEZE_FUZZY_TRIANGULAR;
    output_memberships[0].params[0] = -1.5f;
    output_memberships[0].params[1] = -1.0f;
    output_memberships[0].params[2] = 0.0f;
    output_memberships[0].name = "负";
    
    output_memberships[1].type = BREEZE_FUZZY_TRIANGULAR;
    output_memberships[1].params[0] = -0.5f;
    output_memberships[1].params[1] = 0.0f;
    output_memberships[1].params[2] = 0.5f;
    output_memberships[1].name = "零";
    
    output_memberships[2].type = BREEZE_FUZZY_TRIANGULAR;
    output_memberships[2].params[0] = 0.0f;
    output_memberships[2].params[1] = 1.0f;
    output_memberships[2].params[2] = 1.5f;
    output_memberships[2].name = "正";
    
    /* 设置模糊规则 */
    /* 规则1：如果误差为负且误差变化率为负，则输出为负 */
    rules[0].input1_idx = 0;
    rules[0].input2_idx = 0;
    rules[0].output_idx = 0;
    
    /* 规则2：如果误差为负且误差变化率为零，则输出为负 */
    rules[1].input1_idx = 0;
    rules[1].input2_idx = 1;
    rules[1].output_idx = 0;
    
    /* 规则3：如果误差为负且误差变化率为正，则输出为零 */
    rules[2].input1_idx = 0;
    rules[2].input2_idx = 2;
    rules[2].output_idx = 1;
    
    /* 规则4：如果误差为零且误差变化率为负，则输出为负 */
    rules[3].input1_idx = 1;
    rules[3].input2_idx = 0;
    rules[3].output_idx = 0;
    
    /* 规则5：如果误差为零且误差变化率为零，则输出为零 */
    rules[4].input1_idx = 1;
    rules[4].input2_idx = 1;
    rules[4].output_idx = 1;
    
    /* 规则6：如果误差为零且误差变化率为正，则输出为正 */
    rules[5].input1_idx = 1;
    rules[5].input2_idx = 2;
    rules[5].output_idx = 2;
    
    /* 规则7：如果误差为正且误差变化率为负，则输出为零 */
    rules[6].input1_idx = 2;
    rules[6].input2_idx = 0;
    rules[6].output_idx = 1;
    
    /* 规则8：如果误差为正且误差变化率为零，则输出为正 */
    rules[7].input1_idx = 2;
    rules[7].input2_idx = 1;
    rules[7].output_idx = 2;
    
    /* 规则9：如果误差为正且误差变化率为正，则输出为正 */
    rules[8].input1_idx = 2;
    rules[8].input2_idx = 2;
    rules[8].output_idx = 2;
    
    /* 设置控制器参数 */
    BreezeFuzzyController_SetInput1Memberships(&controller, error_memberships, 3);
    BreezeFuzzyController_SetInput2Memberships(&controller, error_change_memberships, 3);
    BreezeFuzzyController_SetOutputMemberships(&controller, output_memberships, 3);
    BreezeFuzzyController_SetRules(&controller, rules, 9);
    
    /* 测试不同输入组合 */
    printf("误差\t误差变化率\t输出\n");
    
    for (i = 0; i < 5; i++) {
        error = -0.8f + i * 0.4f;
        error_change = -0.5f + i * 0.25f;
        
        output = BreezeFuzzyController_Compute(&controller, error, error_change);
        
        printf("%.2f\t%.2f\t\t%.2f\n", error, error_change, output);
    }
    
    /* 释放资源 */
    BreezeFuzzyController_Free(&controller);
}

/* 自适应控制器示例函数 */
void adaptive_controller_example(void) {
    BreezeAdaptivePID controller;
    float setpoint = 1.0f;
    float measurement, output;
    int i;
    
    printf("\n自适应PID控制器示例\n");
    printf("------------------------------------\n");
    
    /* 初始化自适应PID控制器 */
    BreezeAdaptivePID_Init(
        &controller,
        BREEZE_PID_POSITION,
        0.5f, 0.1f, 0.05f,
        0.1f,
        -1.0f, 1.0f,
        0.1f,
        0.2f,
        5
    );
    
    /* 模拟控制过程 */
    printf("迭代\t测量值\t输出\tKp\tKi\tKd\n");
    
    measurement = 0.0f;
    
    for (i = 0; i < 20; i++) {
        /* 计算控制输出 */
        output = BreezeAdaptivePID_Compute(&controller, setpoint, measurement);
        
        /* 模拟系统响应（一阶系统加扰动） */
        measurement = measurement + 0.1f * output;
        if (i > 10) measurement += 0.05f;  /* 添加扰动 */
        
        /* 每5次迭代输出一次结果 */
        if (i % 5 == 0) {
            printf("%d\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n", 
                   i, measurement, output,
                   controller.pid.kp, controller.pid.ki, controller.pid.kd);
        }
    }
}

/* 状态反馈控制器示例函数 */
void state_feedback_controller_example(void) {
    BreezeStateFeedbackController controller;
    float K[2] = {2.0f, 1.0f};  /* 状态反馈增益 */
    float state[2];             /* 系统状态 [位置, 速度] */
    float output;
    int i;
    
    printf("\n状态反馈控制器示例\n");
    printf("------------------------------------\n");
    
    /* 初始化状态反馈控制器 */
    BreezeStateFeedbackController_Init(&controller, K, 2, 2.0f, -1.0f, 1.0f);
    
    /* 设置参考输入 */
    BreezeStateFeedbackController_SetReference(&controller, 1.0f);
    
    /* 模拟控制过程 */
    printf("迭代\t位置\t速度\t控制输出\n");
    
    state[0] = 0.0f;  /* 初始位置 */
    state[1] = 0.0f;  /* 初始速度 */
    
    for (i = 0; i < 20; i++) {
        /* 计算控制输出 */
        output = BreezeStateFeedbackController_Compute(&controller, state);
        
        /* 模拟系统响应（二阶系统） */
        state[1] = state[1] + 0.1f * output;  /* 速度更新 */
        state[0] = state[0] + 0.1f * state[1];  /* 位置更新 */
        
        /* 每5次迭代输出一次结果 */
        if (i % 5 == 0) {
            printf("%d\t%.2f\t%.2f\t%.2f\n", i, state[0], state[1], output);
        }
    }
}

/* 控制模块示例 */
void control_examples(void) {
    printf("\n控制模块示例\n");
    printf("==============================\n");
    
    fuzzy_controller_example();
    adaptive_controller_example();
    state_feedback_controller_example();
}

int main(void) {
    printf("Breeze框架控制模块示例\n");
    printf("=======================================\n");
    
    control_examples();
    
    return 0;
}
