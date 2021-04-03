# Hands-on-FreeRTOS

Repository contains multiple small projects created while learning various features of FreeRTOS.
The board used for the projects are STM32F407x DISCOVERY but projects can be configured/ported for any hardware.

Projects are based on various RTOS concepts like:

<ul>
<li>FreeRTOS Task Creation, Deletion, Scheduling</li>
<li>FreeRTOS Stack and Heap Management</li>
<li>Queue management like creation, sending, receiving, blocking, etc</li>
<li>Binary/Counting semaphores</li>
<li>Synchronizing between tasks using Semaphores</li>
<li>Synchronization between multiple events and a task using counting semaphores</li>
<li>Mutual exclusion between Tasks using Mutex services and semaphores</li>
<li>Synchronizing between a task and an interrupt using semaphores</li>
<li>FreeRTOS Idle Hook power saving</li>
<li>FreeRTOS task Notify</li>
</ul>

Some projects also includes SEGGER SystemView traces which are very useful in understanding the internal working of a RTOS by visually showing timing informations, context switching, scheduler, SysTick, tasks and other informations.
