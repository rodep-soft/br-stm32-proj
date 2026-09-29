/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
FreeMarker template error (DEBUG mode; use RETHROW in production!):
The following has evaluated to null or missing:
==> RTOS.queues [in template "freertos_typedefs_cmsis_v2.ftl" at line 16, column 12]
----
Tip: It's the step after the last dot that caused this error, not those before it.
----
Tip: If the failing expression is known to legally refer to something that's sometimes null or missing, either specify a default value like myOptionalVar!myDefault, or use [#if myOptionalVar??]when-present[#else]when-missing[/#if]. (These only cover the last step of the expression; to cover the whole expression, use parenthesis: (myOptionalVar.foo)!myDefault, (myOptionalVar.foo)??
----
----
FTL stack trace ("~" means nesting-related):
- Failed at: #list RTOS.queues as queue [in template "freertos_typedefs_cmsis_v2.ftl" at line 16, column 5]
----
Java stack trace (for programmers):
----
freemarker.core.InvalidReferenceException: [... Exception message was already printed; see it above ...]
at freemarker.core.InvalidReferenceException.getInstance(InvalidReferenceException.java:134)
at freemarker.core.Expression.assertNonNull(Expression.java:249)
at freemarker.core.IteratorBlock.acceptWithResult(IteratorBlock.java:104)
at freemarker.core.IteratorBlock.accept(IteratorBlock.java:94)
at freemarker.core.Environment.visit(Environment.java:334)
at freemarker.core.Environment.visit(Environment.java:376)
at freemarker.core.Environment.visitAndTransform(Environment.java:501)
at freemarker.core.CompressedBlock.accept(CompressedBlock.java:42)
at freemarker.core.Environment.visit(Environment.java:334)
at freemarker.core.Environment.process(Environment.java:313)
at freemarker.template.Template.process(Template.java:383)
at com.st.microxplorer.codegenerator.CodeEngine.freemarkerDo(CodeEngine.java:456)
at com.st.microxplorer.codegenerator.CodeEngine.genCode(CodeEngine.java:305)
at com.st.microxplorer.codegenerator.CodeGenerator.generateOutputCode(CodeGenerator.java:6698)
at com.st.microxplorer.codegenerator.CodeGenerator.generateSpecificCode(CodeGenerator.java:5484)
at com.st.microxplorer.codegenerator.CodeGenerator.generateSpecificCodeFile(CodeGenerator.java:1906)
at com.st.microxplorer.codegenerator.CodeGenerator.generateCodeFiles(CodeGenerator.java:2289)
at com.st.microxplorer.codegenerator.CodeGenerator.generateDefaultConfig(CodeGenerator.java:11396)
at com.st.microxplorer.codegenerator.CodeGenerator.generateCode(CodeGenerator.java:1612)
at com.st.microxplorer.plugins.projectmanager.engine.ProjectBuilder.generateCode(ProjectBuilder.java:3469)
at com.st.microxplorer.plugins.projectmanager.engine.ProjectBuilder.createCode(ProjectBuilder.java:2272)
at com.st.microxplorer.plugins.projectmanager.engine.ProjectBuilder.createProject(ProjectBuilder.java:821)
at com.st.microxplorer.plugins.projectmanager.engine.GenerateProjectThread.run(GenerateProjectThread.java:61)
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
FreeMarker template error (DEBUG mode; use RETHROW in production!):
The following has evaluated to null or missing:
==> RTOS.mutexes [in template "freertos_vars_cmsis_v2.ftl" at line 34, column 12]
----
Tip: It's the step after the last dot that caused this error, not those before it.
----
Tip: If the failing expression is known to legally refer to something that's sometimes null or missing, either specify a default value like myOptionalVar!myDefault, or use [#if myOptionalVar??]when-present[#else]when-missing[/#if]. (These only cover the last step of the expression; to cover the whole expression, use parenthesis: (myOptionalVar.foo)!myDefault, (myOptionalVar.foo)??
----
----
FTL stack trace ("~" means nesting-related):
- Failed at: #list RTOS.mutexes as mutex [in template "freertos_vars_cmsis_v2.ftl" at line 34, column 5]
----
Java stack trace (for programmers):
----
freemarker.core.InvalidReferenceException: [... Exception message was already printed; see it above ...]
at freemarker.core.InvalidReferenceException.getInstance(InvalidReferenceException.java:134)
at freemarker.core.Expression.assertNonNull(Expression.java:249)
at freemarker.core.IteratorBlock.acceptWithResult(IteratorBlock.java:104)
at freemarker.core.IteratorBlock.accept(IteratorBlock.java:94)
at freemarker.core.Environment.visit(Environment.java:334)
at freemarker.core.Environment.visit(Environment.java:376)
at freemarker.core.Environment.visitAndTransform(Environment.java:501)
at freemarker.core.CompressedBlock.accept(CompressedBlock.java:42)
at freemarker.core.Environment.visit(Environment.java:334)
at freemarker.core.Environment.process(Environment.java:313)
at freemarker.template.Template.process(Template.java:383)
at com.st.microxplorer.codegenerator.CodeEngine.freemarkerDo(CodeEngine.java:456)
at com.st.microxplorer.codegenerator.CodeEngine.genCode(CodeEngine.java:305)
at com.st.microxplorer.codegenerator.CodeGenerator.generateOutputCode(CodeGenerator.java:6698)
at com.st.microxplorer.codegenerator.CodeGenerator.generateSpecificCode(CodeGenerator.java:5484)
at com.st.microxplorer.codegenerator.CodeGenerator.generateSpecificCodeFile(CodeGenerator.java:1906)
at com.st.microxplorer.codegenerator.CodeGenerator.generateCodeFiles(CodeGenerator.java:2289)
at com.st.microxplorer.codegenerator.CodeGenerator.generateDefaultConfig(CodeGenerator.java:11396)
at com.st.microxplorer.codegenerator.CodeGenerator.generateCode(CodeGenerator.java:1612)
at com.st.microxplorer.plugins.projectmanager.engine.ProjectBuilder.generateCode(ProjectBuilder.java:3469)
at com.st.microxplorer.plugins.projectmanager.engine.ProjectBuilder.createCode(ProjectBuilder.java:2272)
at com.st.microxplorer.plugins.projectmanager.engine.ProjectBuilder.createProject(ProjectBuilder.java:821)
at com.st.microxplorer.plugins.projectmanager.engine.GenerateProjectThread.run(GenerateProjectThread.java:61)

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
FreeMarker template error (DEBUG mode; use RETHROW in production!):
The following has evaluated to null or missing:
==> RTOS.mutexes [in template "freertos_init_c_cmsis_v2.ftl" at line 7, column 12]
----
Tip: It's the step after the last dot that caused this error, not those before it.
----
Tip: If the failing expression is known to legally refer to something that's sometimes null or missing, either specify a default value like myOptionalVar!myDefault, or use [#if myOptionalVar??]when-present[#else]when-missing[/#if]. (These only cover the last step of the expression; to cover the whole expression, use parenthesis: (myOptionalVar.foo)!myDefault, (myOptionalVar.foo)??
----
----
FTL stack trace ("~" means nesting-related):
- Failed at: #list RTOS.mutexes as mutex [in template "freertos_init_c_cmsis_v2.ftl" at line 7, column 5]
----
Java stack trace (for programmers):
----
freemarker.core.InvalidReferenceException: [... Exception message was already printed; see it above ...]
at freemarker.core.InvalidReferenceException.getInstance(InvalidReferenceException.java:134)
at freemarker.core.Expression.assertNonNull(Expression.java:249)
at freemarker.core.IteratorBlock.acceptWithResult(IteratorBlock.java:104)
at freemarker.core.IteratorBlock.accept(IteratorBlock.java:94)
at freemarker.core.Environment.visit(Environment.java:334)
at freemarker.core.Environment.visit(Environment.java:376)
at freemarker.core.Environment.visitAndTransform(Environment.java:501)
at freemarker.core.CompressedBlock.accept(CompressedBlock.java:42)
at freemarker.core.Environment.visit(Environment.java:334)
at freemarker.core.Environment.process(Environment.java:313)
at freemarker.template.Template.process(Template.java:383)
at com.st.microxplorer.codegenerator.CodeEngine.freemarkerDo(CodeEngine.java:456)
at com.st.microxplorer.codegenerator.CodeEngine.genCode(CodeEngine.java:305)
at com.st.microxplorer.codegenerator.CodeGenerator.generateOutputCode(CodeGenerator.java:6698)
at com.st.microxplorer.codegenerator.CodeGenerator.generateSpecificCode(CodeGenerator.java:5484)
at com.st.microxplorer.codegenerator.CodeGenerator.generateSpecificCodeFile(CodeGenerator.java:1906)
at com.st.microxplorer.codegenerator.CodeGenerator.generateCodeFiles(CodeGenerator.java:2289)
at com.st.microxplorer.codegenerator.CodeGenerator.generateDefaultConfig(CodeGenerator.java:11396)
at com.st.microxplorer.codegenerator.CodeGenerator.generateCode(CodeGenerator.java:1612)
at com.st.microxplorer.plugins.projectmanager.engine.ProjectBuilder.generateCode(ProjectBuilder.java:3469)
at com.st.microxplorer.plugins.projectmanager.engine.ProjectBuilder.createCode(ProjectBuilder.java:2272)
at com.st.microxplorer.plugins.projectmanager.engine.ProjectBuilder.createProject(ProjectBuilder.java:821)
at com.st.microxplorer.plugins.projectmanager.engine.GenerateProjectThread.run(GenerateProjectThread.java:61)
}
FreeMarker template error (DEBUG mode; use RETHROW in production!):
The following has evaluated to null or missing:
==> RTOS.timers [in template "freertos_body_user_threads_cmsis_v2.ftl" at line 30, column 12]
----
Tip: It's the step after the last dot that caused this error, not those before it.
----
Tip: If the failing expression is known to legally refer to something that's sometimes null or missing, either specify a default value like myOptionalVar!myDefault, or use [#if myOptionalVar??]when-present[#else]when-missing[/#if]. (These only cover the last step of the expression; to cover the whole expression, use parenthesis: (myOptionalVar.foo)!myDefault, (myOptionalVar.foo)??
----
----
FTL stack trace ("~" means nesting-related):
- Failed at: #list RTOS.timers as timer [in template "freertos_body_user_threads_cmsis_v2.ftl" at line 30, column 5]
----
Java stack trace (for programmers):
----
freemarker.core.InvalidReferenceException: [... Exception message was already printed; see it above ...]
at freemarker.core.InvalidReferenceException.getInstance(InvalidReferenceException.java:134)
at freemarker.core.Expression.assertNonNull(Expression.java:249)
at freemarker.core.IteratorBlock.acceptWithResult(IteratorBlock.java:104)
at freemarker.core.IteratorBlock.accept(IteratorBlock.java:94)
at freemarker.core.Environment.visit(Environment.java:334)
at freemarker.core.Environment.visit(Environment.java:376)
at freemarker.core.Environment.visitAndTransform(Environment.java:501)
at freemarker.core.CompressedBlock.accept(CompressedBlock.java:42)
at freemarker.core.Environment.visit(Environment.java:334)
at freemarker.core.Environment.process(Environment.java:313)
at freemarker.template.Template.process(Template.java:383)
at com.st.microxplorer.codegenerator.CodeEngine.freemarkerDo(CodeEngine.java:456)
at com.st.microxplorer.codegenerator.CodeEngine.genCode(CodeEngine.java:305)
at com.st.microxplorer.codegenerator.CodeGenerator.generateOutputCode(CodeGenerator.java:6698)
at com.st.microxplorer.codegenerator.CodeGenerator.generateSpecificCode(CodeGenerator.java:5484)
at com.st.microxplorer.codegenerator.CodeGenerator.generateSpecificCodeFile(CodeGenerator.java:1906)
at com.st.microxplorer.codegenerator.CodeGenerator.generateCodeFiles(CodeGenerator.java:2289)
at com.st.microxplorer.codegenerator.CodeGenerator.generateDefaultConfig(CodeGenerator.java:11396)
at com.st.microxplorer.codegenerator.CodeGenerator.generateCode(CodeGenerator.java:1612)
at com.st.microxplorer.plugins.projectmanager.engine.ProjectBuilder.generateCode(ProjectBuilder.java:3469)
at com.st.microxplorer.plugins.projectmanager.engine.ProjectBuilder.createCode(ProjectBuilder.java:2272)
at com.st.microxplorer.plugins.projectmanager.engine.ProjectBuilder.createProject(ProjectBuilder.java:821)
at com.st.microxplorer.plugins.projectmanager.engine.GenerateProjectThread.run(GenerateProjectThread.java:61)

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

