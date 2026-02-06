declare module '@theme/Layout' {
  import type {PropsWithChildren, ReactNode} from 'react';

  type LayoutProps = PropsWithChildren<{
    title?: string;
    description?: string;
    children?: ReactNode;
  }>;

  const Layout: (props: LayoutProps) => JSX.Element;
  export default Layout;
}
