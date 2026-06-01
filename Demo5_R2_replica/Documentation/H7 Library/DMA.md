

## DMA Application

- If you are using DMA, you need to make your buffers be declared with macro that is in [H7_system] header file, *DMA_BUFFER* .
- Doing that DMA will have access to RAM_D, and read write to from it.
- Because the [H7Lib1.0],  is allocating 32Kb for *.dma_buffer* linker in RAM_D2.
- If you did not use the macro before variable initialization, DMA will not be able to read live data or correct data.