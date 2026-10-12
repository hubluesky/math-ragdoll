import { defineConfig } from 'vitest/config';

export default defineConfig({
  test: {
    // vitest 单元测试（Node 环境，core 层引擎无关，见设计文档 §0.4）
    include: ['tests/**/*.test.ts'],
    environment: 'node',
    testTimeout: 30000,
  },
});
