// src/pages/index.tsx
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

/* =====================================================
   HERO SECTION
   ===================================================== */
function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro"
            aria-label="Go to Docusaurus Tutorial"
          >
            Docusaurus Tutorial - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

/* =====================================================
   MAIN HOMEPAGE COMPONENT
   ===================================================== */
export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={siteConfig.title}
      description={siteConfig.tagline ?? 'Welcome to our Docusaurus site'}
    >
      {/* Hero / Intro */}
      <HomepageHeader />

      {/* Main Features */}
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
