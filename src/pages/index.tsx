// src/pages/index.tsx
import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import type { JSX } from 'react';
import styles from './index.module.css';

/* =====================================================
   HERO SECTION WITH ANIMATED BACKGROUND
   ===================================================== */
function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <h1 className="hero__title">{siteConfig.title}</h1>
          <p className="hero__subtitle">{siteConfig.tagline}</p>
          <div className={styles.buttons}>
            <Link
              className="button button--secondary button--lg"
              to="/docs/intro"
              aria-label="Go to Robotic Bookshelf"
            >
              🤖 Explore Robotic Bookshelf
            </Link>
            <Link
              className="button button--outline button--lg"
              to="/docs/intro"
              aria-label="Get Started"
            >
              Get Started →
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

/* =====================================================
   STATISTICS COUNTER SECTION
   ===================================================== */
function StatsSection() {
  const stats = [
    { value: '500+', label: 'AI Models', icon: '🧠' },
    { value: '10K+', label: 'Active Users', icon: '👥' },
    { value: '99.9%', label: 'Uptime', icon: '⚡' },
    { value: '24/7', label: 'Support', icon: '💬' },
  ];

  return (
    <section className={styles.statsSection}>
      <div className="container">
        <div className={styles.statsGrid}>
          {stats.map((stat, idx) => (
            <div key={idx} className={styles.statCard}>
              <div className={styles.statIcon}>{stat.icon}</div>
              <div className={styles.statValue}>{stat.value}</div>
              <div className={styles.statLabel}>{stat.label}</div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

/* =====================================================
   AI TECHNOLOGIES CAROUSEL
   ===================================================== */
function TechCarousel() {
  const [currentIndex, setCurrentIndex] = useState(0);
  
  const technologies = [
    {
      title: 'Machine Learning',
      description: 'Advanced ML algorithms for intelligent automation and prediction',
      icon: '🤖',
      color: '#667eea'
    },
    {
      title: 'Natural Language Processing',
      description: 'Understanding and generating human language with state-of-the-art models',
      icon: '💬',
      color: '#764ba2'
    },
    {
      title: 'Computer Vision',
      description: 'Image recognition and visual intelligence powered by deep learning',
      icon: '👁️',
      color: '#f093fb'
    },
    {
      title: 'Neural Networks',
      description: 'Deep learning architectures that mimic human brain functions',
      icon: '🧠',
      color: '#4facfe'
    },
    {
      title: 'Robotics AI',
      description: 'Intelligent automation and autonomous systems for the future',
      icon: '🦾',
      color: '#43e97b'
    }
  ];

  useEffect(() => {
    const timer = setInterval(() => {
      setCurrentIndex((prev) => (prev + 1) % technologies.length);
    }, 4000);
    return () => clearInterval(timer);
  }, []);

  return (
    <section className={styles.carouselSection}>
      <div className="container">
        <h2 className={styles.sectionTitle}>AI Technologies We Cover</h2>
        <div className={styles.carouselContainer}>
          <div 
            className={styles.carouselTrack}
            style={{ transform: `translateX(-${currentIndex * 100}%)` }}
          >
            {technologies.map((tech, idx) => (
              <div key={idx} className={styles.carouselSlide}>
                <div className={styles.techCard} style={{ borderTopColor: tech.color }}>
                  <div className={styles.techIcon}>{tech.icon}</div>
                  <h3>{tech.title}</h3>
                  <p>{tech.description}</p>
                </div>
              </div>
            ))}
          </div>
          <div className={styles.carouselDots}>
            {technologies.map((_, idx) => (
              <button
                key={idx}
                className={clsx(styles.dot, idx === currentIndex && styles.dotActive)}
                onClick={() => setCurrentIndex(idx)}
                aria-label={`Go to slide ${idx + 1}`}
              />
            ))}
          </div>
        </div>
      </div>
    </section>
  );
}

/* =====================================================
   FEATURES GRID WITH ICONS
   ===================================================== */
function FeaturesGrid() {
  const features = [
    {
      icon: '📚',
      title: 'Comprehensive Documentation',
      description: 'Detailed guides and tutorials for all AI concepts and implementations'
    },
    {
      icon: '⚡',
      title: 'Fast Performance',
      description: 'Optimized algorithms ensuring lightning-fast processing speeds'
    },
    {
      icon: '🔒',
      title: 'Secure & Private',
      description: 'Enterprise-grade security with end-to-end encryption'
    },
    {
      icon: '🌐',
      title: 'Cloud Integration',
      description: 'Seamless integration with major cloud platforms'
    },
    {
      icon: '📊',
      title: 'Analytics Dashboard',
      description: 'Real-time insights and performance metrics at your fingertips'
    },
    {
      icon: '🔄',
      title: 'Continuous Updates',
      description: 'Regular updates with the latest AI research and technologies'
    }
  ];

  return (
    <section className={styles.featuresGridSection}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Why Choose Our Platform</h2>
        <div className={styles.featuresGrid}>
          {features.map((feature, idx) => (
            <div key={idx} className={styles.featureCard}>
              <div className={styles.featureIcon}>{feature.icon}</div>
              <h3>{feature.title}</h3>
              <p>{feature.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

/* =====================================================
   PROGRESS BARS - AI CAPABILITIES
   ===================================================== */
function CapabilitiesBars() {
  const capabilities = [
    { name: 'Accuracy', percentage: 98 },
    { name: 'Speed', percentage: 95 },
    { name: 'Scalability', percentage: 92 },
    { name: 'Reliability', percentage: 99 },
  ];

  return (
    <section className={styles.capabilitiesSection}>
      <div className="container">
        <div className={styles.capabilitiesContent}>
          <div className={styles.capabilitiesText}>
            <h2>Platform Performance Metrics</h2>
            <p>
              Our AI platform delivers exceptional performance across all key metrics,
              ensuring you get the best possible results for your projects.
            </p>
          </div>
          <div className={styles.capabilitiesBars}>
            {capabilities.map((capability, idx) => (
              <div key={idx} className={styles.capabilityItem}>
                <div className={styles.capabilityHeader}>
                  <span className={styles.capabilityName}>{capability.name}</span>
                  <span className={styles.capabilityValue}>{capability.percentage}%</span>
                </div>
                <div className={styles.progressBar}>
                  <div 
                    className={styles.progressFill}
                    style={{ width: `${capability.percentage}%` }}
                  />
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>
    </section>
  );
}

/* =====================================================
   TESTIMONIALS SLIDER
   ===================================================== */
function Testimonials() {
  const [activeTestimonial, setActiveTestimonial] = useState(0);
  
  const testimonials = [
    {
      text: "This platform revolutionized how we approach AI development. The documentation is outstanding!",
      author: "Sarah Johnson",
      role: "ML Engineer at TechCorp",
      avatar: "👩‍💻"
    },
    {
      text: "The best AI resource I've found. Comprehensive, up-to-date, and incredibly well-organized.",
      author: "Michael Chen",
      role: "Data Scientist at AI Labs",
      avatar: "👨‍🔬"
    },
    {
      text: "Game-changing platform for AI researchers. It has everything you need in one place.",
      author: "Emily Rodriguez",
      role: "Research Lead at Innovation Hub",
      avatar: "👩‍🎓"
    }
  ];

  return (
    <section className={styles.testimonialsSection}>
      <div className="container">
        <h2 className={styles.sectionTitle}>What Our Users Say</h2>
        <div className={styles.testimonialCard}>
          <div className={styles.testimonialAvatar}>
            {testimonials[activeTestimonial].avatar}
          </div>
          <p className={styles.testimonialText}>"{testimonials[activeTestimonial].text}"</p>
          <div className={styles.testimonialAuthor}>
            <strong>{testimonials[activeTestimonial].author}</strong>
            <span>{testimonials[activeTestimonial].role}</span>
          </div>
        </div>
        <div className={styles.testimonialDots}>
          {testimonials.map((_, idx) => (
            <button
              key={idx}
              className={clsx(styles.dot, idx === activeTestimonial && styles.dotActive)}
              onClick={() => setActiveTestimonial(idx)}
              aria-label={`View testimonial ${idx + 1}`}
            />
          ))}
        </div>
      </div>
    </section>
  );
}

/* =====================================================
   CTA SECTION
   ===================================================== */
function CTASection() {
  return (
    <section className={styles.ctaSection}>
      <div className="container">
        <div className={styles.ctaContent}>
          <h2>Ready to Get Started?</h2>
          <p>Join thousands of developers and researchers using our platform</p>
          <div className={styles.ctaButtons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/intro"
            >
              Start Learning
            </Link>
            <Link
              className="button button--outline button--lg"
              to="/docs/intro"
            >
              View Documentation
            </Link>
          </div>
        </div>
      </div>
    </section>
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
      description={siteConfig.tagline ?? 'Welcome to our AI-powered platform'}
    >
      {/* Hero Section */}
      <HomepageHeader />

      {/* Statistics */}
      <StatsSection />

      {/* Main Features */}
      <main>
        <HomepageFeatures />
      </main>

      {/* AI Technologies Carousel */}
      <TechCarousel />

      {/* Features Grid */}
      <FeaturesGrid />

      {/* Capabilities Progress Bars */}
      <CapabilitiesBars />

      {/* Testimonials */}
      <Testimonials />

      {/* Call to Action */}
      <CTASection />
    </Layout>
  );
}