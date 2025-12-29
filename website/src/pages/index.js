import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container animate-fade-in">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/chapters/module-1-foundations/chapter-1-introduction-to-physical-ai">
            Start Learning 🚀
          </Link>
          <Link
            className="button button--primary button--lg"
            to="/docs/intro">
            Quick Tutorial 📖
          </Link>
        </div>
        <div className={styles.scrollIndicator}></div>
      </div>
    </header>
  );
}

function ModulesSummary() {
  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <Heading as="h2" className="text--center margin-bottom--xl gradient-text">
          Master the Future of Robotics
        </Heading>
        <div className="row">
          <div className="col col--6">
            <div className="glass-card padding--lg margin-bottom--md">
              <Heading as="h3">🧠 Embodied Intelligence</Heading>
              <p>
                Dive deep into how AI interacts with the physical world, moving beyond
                digital brains to physical existence.
              </p>
            </div>
          </div>
          <div className="col col--6">
            <div className="glass-card padding--lg margin-bottom--md">
              <Heading as="h3">⚙️ ROS 2 & Simulation</Heading>
              <p>
                Build the nervous system of your robots using ROS 2 and validate
                designs in high-fidelity Gazebo and Isaac Sim environments.
              </p>
            </div>
          </div>
          <div className="col col--6">
            <div className="glass-card padding--lg margin-bottom--md">
              <Heading as="h3">👁️ Vision-Language-Action</Heading>
              <p>
                Implement cutting-edge VLA pipelines that allow robots to see,
                understand commands, and execute complex physical tasks.
              </p>
            </div>
          </div>
          <div className="col col--6">
            <div className="glass-card padding--lg margin-bottom--md">
              <Heading as="h3">🤖 Humanoid Evolution</Heading>
              <p>
                From simple bipeds to advanced humanoid systems, explore the
                mechanical and algorithmic challenges of human-like forms.
              </p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="Interactive textbook on Physical AI and Humanoid Robotics with AI-powered learning assistance">
      <HomepageHeader />
      <main>
        <ModulesSummary />
        <HomepageFeatures />
      </main>
    </Layout>
  );
}