import type { StorybookConfig } from '@storybook/react-vite';

const config: StorybookConfig = {
  stories: ['../src/**/*.stories.@(js|jsx|mjs|ts|tsx)', './*.stories.tsx'],
  staticDirs: ['./public'],
  addons: [
    '@storybook/addon-onboarding',
    '@storybook/addon-links',
  ],
  framework: '@storybook/react-vite',
  docs: {},
  swc: () => ({
    jsc: {
      transform: {
        react: {
          runtime: 'automatic',
        },
      },
    },
  }),
};
export default config;
