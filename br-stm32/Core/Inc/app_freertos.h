/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.h
  * Description        : FreeRTOS applicative header file
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_FREERTOS_H
#define __APP_FREERTOS_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h" 
#include "cmsis_os2.h"
#n
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Exported macro -------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */
FreeMarker template error (DEBUG mode; use RETHROW in production!):
The following has evaluated to null or missing:
==> RTOS.mutexes [in template "freertos_app_h.ftl" at line 97, column 12]
----
Tip: It's the step after the last dot that caused this error, not those before it.
----
Tip: If the failing expression is known to legally refer to something that's sometimes null or missing, either specify a default value like myOptionalVar!myDefault, or use [#if myOptionalVar??]when-present[#else]when-missing[/#if]. (These only cover the last step of the expression; to cover the whole expression, use parenthesis: (myOptionalVar.foo)!myDefault, (myOptionalVar.foo)??
----
----
FTL stack trace ("~" means nesting-related):
- Failed at: #list RTOS.mutexes as mutex [in template "freertos_app_h.ftl" at line 97, column 5]
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
at freemarker.core.Environment.visit(Environment.java:340)
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