import clsx from 'clsx';
import Heading from '@theme/Heading';
import EnhancedCard from '@site/src/components/EnhancedCard';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Physical AI Fundamentals',
    Svg: require('@site/static/img/robot1.svg').default,
    description: (
      <>
        Learn the core principles of Physical AI - from embodied intelligence to
        real-world interaction and sensorimotor learning.
      </>
    ),
  },
  {
    title: 'Humanoid Robotics',
    Svg: require('@site/static/img/robot2.svg').default,
    description: (
      <>
        Explore advanced humanoid robotics concepts including locomotion,
        manipulation, and human-robot interaction.
      </>
    ),
  },
  {
    title: 'Interactive Learning',
    Svg: require('@site/static/img/robot3.svg').default,
    description: (
      <>
        Engage with the textbook content through our AI-powered chatbot
        that answers questions about the material.
      </>
    ),
  },
];

function Feature({ Svg, title, description }) {
  return (
    <div className={clsx('col col--4', styles.featureCol)}>
      <EnhancedCard
        title={title}
        description={description}
        icon={<Svg className={styles.featureSvg} role="img" />}
        variant="glass"
        hoverEffect={true}
      />
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={clsx(styles.features, 'padding-vert--xl')}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
