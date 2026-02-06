// For Layout
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

// For Link
declare module '@docusaurus/Link' {
  import type {AnchorHTMLAttributes, DetailedHTMLProps, ReactNode} from 'react';

  type LinkProps = DetailedHTMLProps<
    AnchorHTMLAttributes<HTMLAnchorElement>,
    HTMLAnchorElement
  > & { to: string; children?: ReactNode };

  const Link: (props: LinkProps) => JSX.Element;
  export default Link;
}

// For useDocusaurusContext
declare module '@docusaurus/useDocusaurusContext' {
  type DocusaurusContext = {
    siteConfig: Record<string, any>;
  };
  export default function useDocusaurusContext(): DocusaurusContext;
}
